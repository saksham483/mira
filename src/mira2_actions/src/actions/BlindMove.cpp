#include "actions.hpp"
#include <algorithm>

BlindMove::BlindMove(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state) {}

BT::PortsList BlindMove::providedPorts() {
    return {
        BT::InputPort<double>("duration", "Move duration in seconds"),
        BT::InputPort<int>("fwd", 1500, "Forward PWM"),
        BT::InputPort<int>("lat", 1500, "Lateral PWM"),
        BT::InputPort<int>("thr", 1500, "Thrust PWM"),
        BT::InputPort<int>("yaw", 1500, "Yaw PWM")
    };
}

BT::NodeStatus BlindMove::onStart() {
    getInput("duration", duration_);
    getInput("fwd", pwm_fwd_); 
    getInput("lat", pwm_lat_);
    getInput("thr", pwm_thr_); 
    getInput("yaw", pwm_yaw_);
    
    start_time_ = ros_state_->node->now();
    RCLCPP_INFO(ros_state_->node->get_logger(), "[BLIND MOVE] Executing for %.1f s", duration_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BlindMove::onRunning() {
    double elapsed = (ros_state_->node->now() - start_time_).seconds();
    
    if (elapsed >= duration_) {
        RCLCPP_INFO(ros_state_->node->get_logger(), "[BLIND MOVE] Complete.");
        publish_neutral(ros_state_);
        return BT::NodeStatus::SUCCESS;
    }

    custom_msgs::msg::Commands cmd;
    cmd.arm = true; 
    cmd.mode = "ALT_HOLD";
    cmd.forward = std::clamp(pwm_fwd_, 1100, 1900);
    cmd.lateral = std::clamp(pwm_lat_, 1100, 1900);
    cmd.thrust = std::clamp(pwm_thr_, 1100, 1900);
    cmd.yaw = std::clamp(pwm_yaw_, 1100, 1900);
    
    cmd.pitch = 1500; 
    cmd.roll = 1500;
    ros_state_->cmd_publisher->publish(cmd);

    return BT::NodeStatus::RUNNING;
}

void BlindMove::onHalted() {
    publish_neutral(ros_state_);
}