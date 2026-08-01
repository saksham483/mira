#include "actions.hpp"

DiveToDepth::DiveToDepth(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state) {}

BT::PortsList DiveToDepth::providedPorts() {
    return {
        BT::InputPort<double>("target_depth", "Depth to dive to"),
        BT::InputPort<double>("tolerance", 0.05, "Depth error tolerance")
    };
}

BT::NodeStatus DiveToDepth::onStart() {
    getInput("target_depth", target_depth_);
    getInput("tolerance", tolerance_);
    // We send it once here, but we will ALSO send it in onRunning to ensure delivery
    std::string cmd = "DIVE," + std::to_string(target_depth_);
    set_controller_state(ros_state_, cmd);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DiveToDepth::onRunning() {
    // 1. Continuously publish so the simulator catches it after ROS 2 discovery
    std::string cmd_str = "DIVE," + std::to_string(target_depth_);
    set_controller_state(ros_state_, cmd_str);

    // 2. Read the simulated depth
    double current_depth = ros_state_->telemetry.external_pressure;
    double error = target_depth_ - current_depth;

    // 3. Print the status to the terminal every 1 second
    RCLCPP_INFO_THROTTLE(
        ros_state_->node->get_logger(),
        *ros_state_->node->get_clock(),
        1000, 
        "[DIVE] Waiting for depth... Current: %.2f | Target: %.2f", current_depth, target_depth_
    );

    // 4. Check if we reached the target
    if (std::abs(error) < tolerance_) {
        RCLCPP_INFO(ros_state_->node->get_logger(), "[DIVE] Reached target depth!");
        set_controller_state(ros_state_, "IDLE");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void DiveToDepth::onHalted() { set_controller_state(ros_state_, "IDLE"); }