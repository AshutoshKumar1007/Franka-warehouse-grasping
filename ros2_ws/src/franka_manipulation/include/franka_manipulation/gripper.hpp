#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace franka_manipulation
{

class Gripper
{
public:

    explicit Gripper(
        rclcpp::Node& node
    );

    bool initialize();

    bool open();

    bool close();

    bool grasp(
        double width,
        double force = 40.0
    );

    bool stop();

private:

    rclcpp::Node& node_;
};

} // namespace franka_manipulation