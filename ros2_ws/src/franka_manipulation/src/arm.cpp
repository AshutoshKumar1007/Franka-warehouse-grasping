#include "franka_manipulation/arm.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace franka_manipulation
{

Arm::Arm(rclcpp::Node& node)
: node_(node)
{
}

Arm::~Arm()
{
    if (mg_executor_)
    {
        mg_executor_->cancel();
    }
    if (mg_spinner_.joinable())
    {
        mg_spinner_.join();
    }
}

void Arm::initialize()
{
    // -------------------------------------------------------
    // 1. Copy all parameters from the main node
    //    (robot_description, robot_description_semantic,
    //     kinematics, use_sim_time, etc.)
    // -------------------------------------------------------
    std::vector<rclcpp::Parameter> overrides;

    auto params = node_.list_parameters({}, 10);

    for (const auto& name : params.names)
    {
        overrides.push_back(
            node_.get_parameter(name)
        );

        RCLCPP_INFO(
            node_.get_logger(),
            "Copying parameter: %s",
            name.c_str()
        );
    }
    RCLCPP_INFO(
        node_.get_logger(),
        "Copied %zu parameters.",
        overrides.size()
    );

    // -------------------------------------------------------
    // 2. Create a DEDICATED node for MoveGroupInterface
    //    This avoids the executor binding conflict that
    //    causes plan() to deadlock.
    // -------------------------------------------------------
    auto options = rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true)
        .parameter_overrides(overrides)
        .use_global_arguments(false);

    mg_node_ = std::make_shared<rclcpp::Node>(
        "move_group_client",
        node_.get_namespace(),
        options
    );

    // -------------------------------------------------------
    // 3. Spin the dedicated node in its own thread
    //    (exactly the MoveIt2 tutorial pattern)
    // -------------------------------------------------------
    mg_executor_ = std::make_shared<
        rclcpp::executors::SingleThreadedExecutor
    >();
    mg_executor_->add_node(mg_node_);

    mg_spinner_ = std::thread(
        [this]()
        {
            mg_executor_->spin();
        }
    );

    // -------------------------------------------------------
    // 4. Create MoveGroupInterface on the dedicated node
    // -------------------------------------------------------
    move_group_ =
        std::make_unique<
            moveit::planning_interface::MoveGroupInterface
        >(
            mg_node_,
            "fr3_arm"
        );

    // Wait for the state monitor to receive actual
    // joint positions from the sim before we proceed.
    if (!move_group_->startStateMonitor())
    {
        RCLCPP_WARN(
            node_.get_logger(),
            "Timed out waiting for current state."
        );
    }
    RCLCPP_INFO(
        node_.get_logger(),
        "MoveGroupInterface initialized."
    );

    // -------------------------------------------------------
    // 5. Force planning to target the gripper TCP, not the
    //    group's default tip link (fr3_link8).
    //
    //    Grasp candidates describe where the FINGERS should
    //    end up. If we plan for fr3_link8 instead, MoveIt
    //    drives the wrist ~10cm too far along the approach
    //    axis (the hand+finger offset), which is exactly the
    //    kind of thing that turns every IK solution near the
    //    table/object into a collision -> "Unable to sample
    //    any valid states for goal tree".
    // -------------------------------------------------------
    const std::string ee_link = "fr3_hand_tcp";
    move_group_->setEndEffectorLink(ee_link);

    RCLCPP_INFO(
        node_.get_logger(),
        "Planning frame : %s",
        move_group_->getPlanningFrame().c_str()
    );

    RCLCPP_INFO(
        node_.get_logger(),
        "End effector   : %s",
        move_group_->getEndEffectorLink().c_str()
    );

    if (move_group_->getEndEffectorLink() != ee_link)
    {
        RCLCPP_ERROR(
            node_.get_logger(),
            "Requested EE link '%s' was NOT accepted (still '%s'). "
            "That link probably doesn't exist in this URDF/SRDF. "
            "Check with: ros2 run tf2_tools view_frames (or grep "
            "the fr3 xacro for 'hand_tcp'). Until this matches, "
            "goal poses are being planned for the WRIST, not the "
            "fingertips.",
            ee_link.c_str(),
            move_group_->getEndEffectorLink().c_str()
        );
    }

    // Reasonable defaults for a first working run.
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    // -------------------------------------------------------
    // 6. TF buffer for transforming grasp poses into the
    //    planning frame before Cartesian planning (unlike
    //    setPoseTarget(PoseStamped), computeCartesianPath only
    //    accepts plain Pose in the planning frame).
    // -------------------------------------------------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(mg_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// geometry_msgs::msg::PoseStamped
// Arm::get_current_pose() const
// {
//     return move_group_->getCurrentPose();
// }
geometry_msgs::msg::PoseStamped
Arm::get_current_pose() const
{
    // Ensure we have a fresh state (wait up to 5s)
    move_group_->getCurrentState(5.0);
    return move_group_->getCurrentPose();
}

bool Arm::move_to_pose(
    const geometry_msgs::msg::PoseStamped& pose
)
{
    RCLCPP_INFO(
        node_.get_logger(),
        "Planning motion..."
    );
    move_group_->setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto success =
        move_group_->plan(plan) ==
        moveit::core::MoveItErrorCode::SUCCESS;

    RCLCPP_INFO(
        node_.get_logger(),
        "Plan complete (success=%s).",
        success ? "true" : "false"
    );

    if (!success)
    {
        RCLCPP_ERROR(
            node_.get_logger(),
            "Planning failed."
        );
        return false;
    }

    RCLCPP_INFO(
        node_.get_logger(),
        "Plan successful. Executing..."
    );

    auto result = move_group_->execute(plan);

    RCLCPP_INFO(
        node_.get_logger(),
        "Execution finished."
    );

    return result ==
           moveit::core::MoveItErrorCode::SUCCESS;
}

geometry_msgs::msg::Pose
Arm::transform_to_planning_frame(
    const geometry_msgs::msg::PoseStamped& pose_in
)
{
    const std::string planning_frame =
        move_group_->getPlanningFrame();

    if (pose_in.header.frame_id.empty() ||
        pose_in.header.frame_id == planning_frame)
    {
        return pose_in.pose;
    }

    try
    {
        geometry_msgs::msg::PoseStamped out;
        tf_buffer_->transform(
            pose_in,
            out,
            planning_frame,
            tf2::durationFromSec(0.5)
        );
        return out.pose;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(
            node_.get_logger(),
            "TF transform '%s' -> '%s' failed: %s. "
            "Using pose unmodified — this is almost certainly wrong.",
            pose_in.header.frame_id.c_str(),
            planning_frame.c_str(),
            ex.what()
        );
        return pose_in.pose;
    }
}

bool Arm::move_cartesian(
    const geometry_msgs::msg::PoseStamped& target,
    bool avoid_collisions,
    double eef_step
)
{
    std::vector<geometry_msgs::msg::Pose> waypoints{
        transform_to_planning_frame(target)
    };

    move_group_->setStartStateToCurrentState();

    moveit_msgs::msg::RobotTrajectory trajectory;

    // NOTE: older MoveIt releases had a jump_threshold argument
    // between eef_step and trajectory:
    //   computeCartesianPath(waypoints, eef_step, jump_threshold,
    //                         trajectory, avoid_collisions)
    // If this line fails to compile against your MoveIt version,
    // add `0.0,` (jump_threshold disabled) before `trajectory`.
    double fraction = move_group_->computeCartesianPath(
        waypoints,
        eef_step,
        0.0,              // jump_threshold — disabled
        trajectory,
        avoid_collisions
    );

    RCLCPP_INFO(
        node_.get_logger(),
        "Cartesian path: %.1f%% complete (avoid_collisions=%s).",
        fraction * 100.0,
        avoid_collisions ? "true" : "false"
    );

    if (fraction < 0.95)
    {
        RCLCPP_ERROR(
            node_.get_logger(),
            "Cartesian approach incomplete (%.1f%%). Not executing.",
            fraction * 100.0
        );
        return false;
    }

    auto result = move_group_->execute(trajectory);

    return result ==
           moveit::core::MoveItErrorCode::SUCCESS;
}

std::string Arm::get_end_effector_link() const
{
    return move_group_->getEndEffectorLink();
}

} // namespace franka_manipulation