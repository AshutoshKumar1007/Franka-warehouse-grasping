#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace franka_manipulation
{

struct GraspCandidate
{
    geometry_msgs::msg::PoseStamped pose;

    double width = 0.08;

    double score = 1.0;
};

struct PickPlaceGoal
{
    GraspCandidate grasp;

    geometry_msgs::msg::PoseStamped place_pose;
};

} // namespace franka_manipulation