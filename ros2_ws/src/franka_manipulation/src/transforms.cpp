#include "franka_manipulation/transforms.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

namespace franka_manipulation
{

geometry_msgs::msg::PoseStamped
compute_pregrasp_pose(
    const geometry_msgs::msg::PoseStamped& grasp_pose,
    double offset
)
{
    // Back off along the GRASP's own approach axis, not world Z.
    //
    // Convention assumed here (standard for GraspNet / Contact-
    // GraspNet / AnyGrasp-style candidates): the grasp frame's
    // local +Z axis points from the wrist toward the object,
    // i.e. it's the direction the gripper drives IN to grasp.
    // Backing off means moving along -Z of that same frame.
    //
    // If, after this fix, the arm still drives further into the
    // object/table instead of retreating, your generator uses the
    // opposite sign convention — flip `approach_axis` below to
    // `-T.rotation().col(2)`, or check the frame convention docs
    // for whichever grasp network you're using.
    Eigen::Isometry3d T;
    tf2::fromMsg(grasp_pose.pose, T);

    Eigen::Vector3d approach_axis = T.rotation().col(2);

    Eigen::Isometry3d T_pregrasp = T;
    T_pregrasp.translation() -= approach_axis * offset;

    geometry_msgs::msg::PoseStamped pregrasp = grasp_pose;
    pregrasp.pose = tf2::toMsg(T_pregrasp);

    return pregrasp;
}

///////////////////////////////////////////////////////////////////////////////

geometry_msgs::msg::PoseStamped
compute_lift_pose(
    const geometry_msgs::msg::PoseStamped& grasp_pose,
    double lift_distance
)
{
    // Lifting is deliberately kept in world Z (straight up),
    // regardless of the grasp orientation.
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