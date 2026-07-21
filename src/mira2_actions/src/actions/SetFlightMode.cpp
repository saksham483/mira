//this action make the bot leave uncontrolled for 0.25 seconds so consider it during the planning of the .xml file
#include "actions.hpp"

SetFlightMode::SetFlightMode(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state) {}

BT::PortsList SetFlightMode::providedPorts() {
    return { BT::InputPort<std::string>("mode", "Target flight mode (MANUAL or ALT_HOLD)") };
}

BT::NodeStatus SetFlightMode::onStart() {
    getInput("mode", target_mode_);
    start_time_ = ros_state_->node->now();
    RCLCPP_INFO(ros_state_->node->get_logger(), "[MODE SWITCH] Disarming to set mode: %s", target_mode_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetFlightMode::onRunning() {
    double elapsed = (ros_state_->node->now() - start_time_).seconds();

    // Command disarm and the new mode
    custom_msgs::msg::Commands cmd;
    cmd.arm = false;               // MUST be false to change mode in master.py
    cmd.mode = target_mode_;
    cmd.forward = 1500; cmd.lateral = 1500; cmd.thrust = 1500;
    cmd.yaw = 1500; cmd.pitch = 1500; cmd.roll = 1500;
    cmd.servo1 = 1500; cmd.servo2 = 1500;
    
    ros_state_->cmd_publisher->publish(cmd);

    // Wait 0.25 seconds to ensure Pixhawk registers the mode switch before we move to the next node and re-arm
    if (elapsed >= 0.25) {
        RCLCPP_INFO(ros_state_->node->get_logger(), "[MODE SWITCH] Mode %s set successfully.", target_mode_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void SetFlightMode::onHalted() {
    // Nothing special needed on halt
}
