#include "franka_manipulation/executor.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace franka_manipulation
{

ExecutorNode::ExecutorNode()
: Node(
    "executor",
    rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true)
)
{
    RCLCPP_INFO(
        get_logger(),
        "Franka Manipulation Executor started."
    );

    executor_ = std::make_unique<PickPlaceExecutor>(*this);

    init_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
            init_timer_->cancel();

            std::thread(
                [this]()
                {
                    initialize();
                }
            ).detach();
        });
}

void ExecutorNode::initialize()
{
    if (!executor_->initialize())
    {
        RCLCPP_FATAL(
            get_logger(),
            "Failed to initialize manipulation pipeline."
        );
        return;
    }

    auto qos =
        rclcpp::QoS(1)
            .reliable();

    grasp_sub_ =
        create_subscription<
            franka_interfaces::msg::GraspCandidate>(
            "/selected_grasp",
            qos,
            std::bind(
                &ExecutorNode::grasp_callback,
                this,
                std::placeholders::_1
            )
        );

    RCLCPP_INFO(
        get_logger(),
        "Waiting for grasp candidates..."
    );
}

void ExecutorNode::grasp_callback(
    const franka_interfaces::msg::GraspCandidate::SharedPtr msg
)
{
    RCLCPP_INFO(
        get_logger(),
        "Received grasp candidate."
    );

    executor_->execute_pick_place(
        *msg
    );
}

///////////////////////////////////////////////////////////////////////////////

PickPlaceExecutor::PickPlaceExecutor(
    rclcpp::Node& node
)
: node_(node)
{
    arm_ =
        std::make_unique<Arm>(
            node_
        );

    gripper_ =
        std::make_unique<Gripper>(
            node_
        );
}

bool PickPlaceExecutor::initialize()
{
    arm_->initialize();
    gripper_->initialize();
    RCLCPP_INFO(
        node_.get_logger(),
        "Manipulator ready."
    );

    return true;
}

///////////////////////////////////////////////////////////////////////////////

bool PickPlaceExecutor::execute_pick_place(
    const franka_interfaces::msg::GraspCandidate& grasp
)
{
    RCLCPP_INFO(
        node_.get_logger(),
        "Executing grasp candidate."
    );

    if (!gripper_->open())
    {
        return false;
    }

    if (!execute_pregrasp(grasp))
    {
        RCLCPP_ERROR(
            node_.get_logger(),
            "Failed to reach pre-grasp pose. Stopping here — "
            "this is the reachability test failing."
        );
        return false;
    }

    if (!execute_approach(grasp))
    {
        RCLCPP_ERROR(
            node_.get_logger(),
            "Cartesian approach into grasp pose failed."
        );
        return false;
    }

    // Gripper is currently a stub (always returns true) — this
    // will start doing something once it's wired to the real
    // Franka gripper action server.
    // NOTE: verify `grasp.width` matches your actual message
    // field name (`ros2 interface show franka_interfaces/msg/GraspCandidate`).
    if (!gripper_->grasp(grasp.width))
    {
        return false;
    }

    if (!execute_lift(grasp))
    {
        RCLCPP_ERROR(
            node_.get_logger(),
            "Lift failed."
        );
        return false;
    }

    RCLCPP_INFO(
        node_.get_logger(),
        "Grasp candidate reached, approached, and lifted "
        "successfully. (Place stage not wired in yet — "
        "grasp_callback only carries a GraspCandidate, no "
        "place pose.)"
    );

    return true;
}

///////////////////////////////////////////////////////////////////////////////

bool PickPlaceExecutor::execute_pregrasp(
    const franka_interfaces::msg::GraspCandidate& grasp
)
{
    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header = grasp.header;
    grasp_pose.pose   = grasp.pose;

    auto pregrasp_pose = compute_pregrasp_pose(grasp_pose, 0.10);

    RCLCPP_INFO(
        node_.get_logger(),
        "Moving to pre-grasp pose (10cm standoff along the "
        "grasp's own approach axis)."
    );

    // Full free-space, collision-checked plan. This is the pose
    // that has to be reachable and clear of everything — it's
    // the one the earlier OMPL error was actually failing on,
    // since the previous code skipped straight to grasp.pose.
    return arm_->move_to_pose(pregrasp_pose);
}

///////////////////////////////////////////////////////////////////////////////

bool PickPlaceExecutor::execute_approach(
    const franka_interfaces::msg::GraspCandidate& grasp
)
{
    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header = grasp.header;
    grasp_pose.pose   = grasp.pose;

    RCLCPP_INFO(
        node_.get_logger(),
        "Approaching grasp pose (Cartesian, collision checking "
        "off for this segment)."
    );

    // The grasp pose is *supposed* to have the fingers around/
    // against the object, so collision checking is off here on
    // purpose. This is safe-ish specifically because the segment
    // is short (the 10cm standoff we just backed off from) and
    // dead straight — not a free-space plan through the scene.
    return arm_->move_cartesian(
        grasp_pose,
        /*avoid_collisions=*/false
    );
}

///////////////////////////////////////////////////////////////////////////////

bool PickPlaceExecutor::execute_lift(
    const franka_interfaces::msg::GraspCandidate& grasp
)
{
    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header = grasp.header;
    grasp_pose.pose   = grasp.pose;

    auto lift_pose = compute_lift_pose(grasp_pose, 0.10);

    RCLCPP_INFO(
        node_.get_logger(),
        "Lifting object 10cm."
    );

    // Collisions off here too: the object isn't attached to the
    // planning scene as an attached collision object yet, so
    // MoveIt still sees it sitting in the gripper as a scene
    // obstacle. Attaching/detaching it properly is the next
    // piece to add once reachability itself is confirmed.
    return arm_->move_cartesian(
        lift_pose,
        /*avoid_collisions=*/false
    );
}

} // namespace franka_manipulation