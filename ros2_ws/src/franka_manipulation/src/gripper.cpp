#include "franka_manipulation/gripper.hpp"

namespace franka_manipulation
{

Gripper::Gripper(
    rclcpp::Node& node
)
: node_(node)
{
}

bool Gripper::initialize()
{
    RCLCPP_INFO(
        node_.get_logger(),
        "Gripper initialized."
    );

    return true;
}

bool Gripper::open()
{
    RCLCPP_INFO(
        node_.get_logger(),
        "Opening gripper."
    );

    return true;
}

bool Gripper::close()
{
    RCLCPP_INFO(
        node_.get_logger(),
        "Closing gripper."
    );

    return true;
}

bool Gripper::grasp(
    double width,
    double force
)
{
    RCLCPP_INFO(
        node_.get_logger(),
        "Grasp width %.3f force %.1f",
        width,
        force
    );

    return true;
}

bool Gripper::stop()
{
    RCLCPP_INFO(
        node_.get_logger(),
        "Stopping gripper."
    );

    return true;
}

} // namespace franka_manipulation