#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "franka_interfaces/msg/grasp_candidate.hpp"

#include "franka_manipulation/arm.hpp"
#include "franka_manipulation/gripper.hpp"
#include "franka_manipulation/transforms.hpp"
#include "franka_manipulation/types.hpp"

namespace franka_manipulation
{

class PickPlaceExecutor
{
public:

    explicit PickPlaceExecutor(
        rclcpp::Node& node
    );

    bool initialize();

    bool execute_pick_place(
        const franka_interfaces::msg::GraspCandidate& grasp
    );

private:

    bool execute_pregrasp(
        const franka_interfaces::msg::GraspCandidate& grasp
    );

    bool execute_approach(
        const franka_interfaces::msg::GraspCandidate& grasp
    );

    bool execute_lift(
        const franka_interfaces::msg::GraspCandidate& grasp
    );

    rclcpp::Node& node_;

    std::unique_ptr<Arm> arm_;

    std::unique_ptr<Gripper> gripper_;
};

///////////////////////////////////////////////////////////////////////////////

class ExecutorNode : public rclcpp::Node
{
public:

    ExecutorNode();

private:

    void initialize();

    void grasp_callback(
        const franka_interfaces::msg::GraspCandidate::SharedPtr msg
    );

private:

    std::unique_ptr<PickPlaceExecutor> executor_;

    rclcpp::TimerBase::SharedPtr init_timer_;

    rclcpp::Subscription<
        franka_interfaces::msg::GraspCandidate
    >::SharedPtr grasp_sub_;
};

} // namespace franka_manipulation