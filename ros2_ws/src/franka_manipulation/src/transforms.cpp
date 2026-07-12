#include "franka_manipulation/transforms.hpp"

namespace franka_manipulation
{

geometry_msgs::msg::PoseStamped
compute_pregrasp_pose(
    const geometry_msgs::msg::PoseStamped& grasp_pose,
    double offset
)
{
    auto pose = grasp_pose;

    //
    // Temporary implementation.
    //
    // Contact-GraspNet already predicts the hand pose.
    // For now, back off along world Z.
    //

    pose.pose.position.z += offset;

    return pose;
}

///////////////////////////////////////////////////////////////////////////////

geometry_msgs::msg::PoseStamped
compute_lift_pose(
    const geometry_msgs::msg::PoseStamped& grasp_pose,
    double lift_distance
)
{
    auto pose = grasp_pose;

    pose.pose.position.z += lift_distance;

    return pose;
}

///////////////////////////////////////////////////////////////////////////////

geometry_msgs::msg::PoseStamped
compute_place_pose(
    const geometry_msgs::msg::PoseStamped& place_pose
)
{
    return place_pose;
}

} // namespace franka_manipulation