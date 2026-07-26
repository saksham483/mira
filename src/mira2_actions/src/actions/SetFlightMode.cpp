#include "actions.hpp"

SetFlightMode::SetFlightMode(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state) {}

BT::PortsList SetFlightMode::providedPorts() {
    return { BT::InputPort<std::string>("mode", "STABILIZE or MANUAL") };
}

BT::NodeStatus SetFlightMode::onStart() {
    getInput("mode", target_mode_);
    start_time_ = ros_state_->node->now();
    set_controller_state(ros_state_, "IDLE"); 
    
    RCLCPP_INFO(ros_state_->node->get_logger(), "[MODE] Disarming to set mode: %s", target_mode_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetFlightMode::onRunning() {
    double elapsed = (ros_state_->node->now() - start_time_).seconds();

    custom_msgs::msg::Commands cmd;
    cmd.arm = false;
    cmd.mode = target_mode_;
    cmd.forward = 1500; cmd.lateral = 1500; cmd.thrust = 1500;
    cmd.yaw = 1500; cmd.pitch = 1500; cmd.roll = 1500;
    ros_state_->cmd_publisher->publish(cmd);

    if (elapsed >= 0.3) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void SetFlightMode::onHalted() {
    // nothing is needed here for now
}