#pragma once

#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

namespace franka_manipulation
{

class Arm
{
public:

    explicit Arm(rclcpp::Node& node);

    ~Arm();

    void initialize();

    geometry_msgs::msg::PoseStamped get_current_pose() const;
    
    bool move_to_pose(
        const geometry_msgs::msg::PoseStamped& pose
    );

private:

    rclcpp::Node& node_;

    // Dedicated node + executor for MoveGroupInterface
    // so it doesn't conflict with the main node's executor.
    rclcpp::Node::SharedPtr mg_node_;
    std::shared_ptr<
        rclcpp::executors::SingleThreadedExecutor
    > mg_executor_;
    std::thread mg_spinner_;

    std::unique_ptr<
        moveit::planning_interface::MoveGroupInterface
    > move_group_;
};

} // namespace franka_manipulation