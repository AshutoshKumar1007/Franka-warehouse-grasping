#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "moveit_smoke_test",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    RCLCPP_INFO(node->get_logger(), "Waiting for MoveIt...");

    std::this_thread::sleep_for(2s);

    moveit::planning_interface::MoveGroupInterface move_group(
        node,
        "fr3_arm"
    );

    RCLCPP_INFO(node->get_logger(), "MoveGroup created.");

    auto pose = move_group.getCurrentPose();

    RCLCPP_INFO(
        node->get_logger(),
        "Current Pose: %.3f %.3f %.3f",
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z
    );

    pose.pose.position.x += 0.02;

    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.setPoseTarget(pose);

    RCLCPP_INFO(node->get_logger(), "Planning...");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto result = move_group.plan(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed.");

        executor.cancel();
        spinner.join();

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Executing...");

    result = move_group.execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "SUCCESS");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Execution failed.");
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}