#include "actions.hpp"
#include <algorithm>

DiveToDepth::DiveToDepth(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state), depth_pid_(name+"_pid", ros_state->node) {}

BT::PortsList DiveToDepth::providedPorts() {
    return {
        BT::InputPort<double>("target_depth", "Target pressure/depth value"),
        BT::InputPort<double>("tolerance", 0.05, "Depth error tolerance"),
        BT::InputPort<double>("kp", 5.0, "Depth Kp"),
        BT::InputPort<double>("kd", 10.0, "Depth Kd")
    };
}

BT::NodeStatus DiveToDepth::onStart() {
    getInput("target_depth", target_depth_);
    getInput("tolerance", tolerance_);
    depth_pid_.kp = getInput<double>("kp").value();
    depth_pid_.kd = getInput<double>("kd").value();
    depth_pid_.ki = 0.0;
    depth_pid_.base_offset = 1500;
    depth_pid_.emptyError();
    last_time_ = ros_state_->node->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DiveToDepth::onRunning() {
    rclcpp::Time now = ros_state_->node->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    double current_depth = ros_state_->telemetry.external_pressure;
    double error = target_depth_ - current_depth;

    if (std::abs(error) < tolerance_) {
        publish_neutral(ros_state_);
        return BT::NodeStatus::SUCCESS;
    }

    float thrust_pwm = depth_pid_.pid_control(error, dt, false);
    thrust_pwm = std::clamp(thrust_pwm, 1100.0f, 1900.0f);

    custom_msgs::msg::Commands cmd;
    cmd.arm = true; cmd.mode = "MANUAL";
    cmd.forward = 1500; cmd.lateral = 1500; cmd.yaw = 1500;
    cmd.thrust = static_cast<int>(thrust_pwm);
    ros_state_->cmd_publisher->publish(cmd);

    return BT::NodeStatus::RUNNING;
}

void DiveToDepth::onHalted() { 
    publish_neutral(ros_state_);
    depth_pid_.emptyError(); 
}
//It uses a PID controller on the thrust (Z-axis). It calculates the error between the target_depth and the current_depth from telemetry. It outputs a PWM value (e.g., <1500 to dive, >1500 to rise)