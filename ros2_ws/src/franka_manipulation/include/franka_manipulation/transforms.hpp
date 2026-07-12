#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace franka_manipulation
{

geometry_msgs::msg::PoseStamped
compute_pregrasp_pose(
    const geometry_msgs::msg::PoseStamped& grasp_pose,
    double offset = 0.10
);

geometry_msgs::msg::PoseStamped
compute_lift_pose(
    const geometry_msgs::msg::PoseStamped& grasp_pose,
    double lift_distance = 0.10
);

geometry_msgs::msg::PoseStamped
compute_place_pose(
    const geometry_msgs::msg::PoseStamped& place_pose
);

} // namespace franka_manipulation