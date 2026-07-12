#pragma once

#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace franka_manipulation
{

class Arm
{
public:

    explicit Arm(rclcpp::Node& node);

    ~Arm();

    void initialize();

    geometry_msgs::msg::PoseStamped get_current_pose() const;

    // Free-space plan + execute (goes through the full pipeline,
    // collision-checked). Use for pre-grasp / retreat poses that
    // should stay clear of everything.
    bool move_to_pose(
        const geometry_msgs::msg::PoseStamped& pose
    );

    // Straight-line Cartesian move to a single target pose.
    // avoid_collisions=false is meant for the final few
    // centimeters into a grasp, where the goal pose is expected
    // to be "in collision" with the object by construction.
    bool move_cartesian(
        const geometry_msgs::msg::PoseStamped& target,
        bool avoid_collisions = true,
        double eef_step = 0.005
    );

    std::string get_end_effector_link() const;

private:

    geometry_msgs::msg::Pose transform_to_planning_frame(
        const geometry_msgs::msg::PoseStamped& pose_in
    );

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

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace franka_manipulation