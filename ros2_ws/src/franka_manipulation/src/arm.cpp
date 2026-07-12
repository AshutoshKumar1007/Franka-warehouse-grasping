#include "franka_manipulation/arm.hpp"

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

} // namespace franka_manipulation