#include "actions.hpp"

EmergencySurface::EmergencySurface(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state) {}

BT::PortsList EmergencySurface::providedPorts() { 
    return {}; 
}

BT::NodeStatus EmergencySurface::onStart() {
    RCLCPP_ERROR(ros_state_->node->get_logger(), "[EMERGENCY] Initiating positive buoyancy breach!");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus EmergencySurface::onRunning() {
    custom_msgs::msg::Commands cmd;
    cmd.arm = true; 
    cmd.mode = "MANUAL"; 
    
    cmd.forward = 1500; 
    cmd.lateral = 1500; 
    cmd.yaw = 1500;
    cmd.pitch = 1500;
    cmd.roll = 1500;
    cmd.servo1 = 1500;
    cmd.servo2 = 1500;
    // Send maximum upward thrust commands. 
    // In typical ArduSub configurations, values >1500 translate to upward thrust.
    cmd.thrust = 1900; 
    ros_state_->cmd_publisher->publish(cmd);
    // Run indefinitely. In an emergency, the AUV should try to surface until it is 
    // physically retrieved and disarmed by hardware killswitch.
    return BT::NodeStatus::RUNNING;
}

void EmergencySurface::onHalted() {
    publish_neutral(ros_state_);
}