#include "conditions.hpp"

IsEmergency::IsEmergency(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::ConditionNode(name, config), ros_state_(ros_state) {}

BT::PortsList IsEmergency::providedPorts() {
    return { BT::InputPort<bool>("enable_emergency", false, "Toggle hardware checks") };
}

BT::NodeStatus IsEmergency::tick() {
    bool enable = false;
    getInput("enable_emergency", enable);
    if (!enable) return BT::NodeStatus::FAILURE;

    if (ros_state_->telemetry.battery_voltage > 0.0 && ros_state_->telemetry.battery_voltage < 13.0) {
        RCLCPP_ERROR(ros_state_->node->get_logger(), "[EMERGENCY] LOW BATTERY: %.2fV", ros_state_->telemetry.battery_voltage);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE; 
}
//this is just a placeholder for now, not for actual use 