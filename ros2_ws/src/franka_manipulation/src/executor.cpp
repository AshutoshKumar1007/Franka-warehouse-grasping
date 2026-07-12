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

// bool PickPlaceExecutor::execute_pick_place(
//     const PickPlaceGoal& goal
// )
// {
//     RCLCPP_INFO(
//         node_.get_logger(),
//         "Starting pick-and-place."
//     );

//     if (!gripper_->open())
//     {
//         return false;
//     }

//     if (!execute_pregrasp(goal.grasp))
//     {
//         return false;
//     }

//     if (!execute_approach(goal.grasp))
//     {
//         return false;
//     }

//     if (!gripper_->grasp(goal.grasp.width))
//     {
//         return false;
//     }

//     if (!execute_lift(goal.grasp))
//     {
//         return false;
//     }

//     if (!execute_place(goal.place_pose))
//     {
//         return false;
//     }

//     if (!gripper_->open())
//     {
//         return false;
//     }

//     RCLCPP_INFO(
//         node_.get_logger(),
//         "Pick-and-place completed successfully."
//     );

//     return true;
// }

bool PickPlaceExecutor::execute_pick_place(
    const franka_interfaces::msg::GraspCandidate& grasp
)
{
    RCLCPP_INFO(
        node_.get_logger(),
        "Executing grasp candidate."
    );

    geometry_msgs::msg::PoseStamped target;

    target.header = grasp.header;
    target.pose   = grasp.pose;

    return arm_->move_to_pose(
        target
    );
}

///////////////////////////////////////////////////////////////////////////////

// bool PickPlaceExecutor::execute_pregrasp(
//     const GraspCandidate& grasp
// )
// {
//     RCLCPP_INFO(
//         node_.get_logger(),
//         "Moving to pre-grasp pose."
//     );

//     auto pregrasp_pose =
//         compute_pregrasp_pose(
//             grasp.pose
//         );

//     return arm_->move_to_pose(
//         pregrasp_pose
//     );
// }

// ///////////////////////////////////////////////////////////////////////////////

// bool PickPlaceExecutor::execute_approach(
//     const GraspCandidate& grasp
// )
// {
//     RCLCPP_INFO(
//         node_.get_logger(),
//         "Approaching grasp pose."
//     );

//     return arm_->move_to_pose(
//         grasp.pose
//     );
// }

// ///////////////////////////////////////////////////////////////////////////////

// bool PickPlaceExecutor::execute_lift(
//     const GraspCandidate& grasp
// )
// {
//     RCLCPP_INFO(
//         node_.get_logger(),
//         "Lifting object."
//     );

//     auto lift_pose =
//         compute_lift_pose(
//             grasp.pose
//         );

//     return arm_->move_to_pose(
//         lift_pose
//     );
// }

// ///////////////////////////////////////////////////////////////////////////////

// bool PickPlaceExecutor::execute_place(
//     const geometry_msgs::msg::PoseStamped& place_pose
// )
// {
//     RCLCPP_INFO(
//         node_.get_logger(),
//         "Moving to place pose."
//     );

//     return arm_->move_to_pose(
//         place_pose
//     );
// }

} // namespace franka_manipulation