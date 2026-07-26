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
    std::string cmd = "DIVE," + std::to_string(target_depth_);
    set_controller_state(ros_state_, cmd);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DiveToDepth::onRunning() {
    double current_depth = ros_state_->telemetry.external_pressure;
    double error = target_depth_ - current_depth;

    if (std::abs(error) < tolerance_) {
        RCLCPP_INFO(ros_state_->node->get_logger(), "[DIVE] Reached target depth!");
        set_controller_state(ros_state_, "IDLE");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void DiveToDepth::onHalted() { set_controller_state(ros_state_, "IDLE"); }