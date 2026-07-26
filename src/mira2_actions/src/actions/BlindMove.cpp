#include "actions.hpp"

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
    
    int fwd, lat, thr, yaw;
    getInput("fwd", fwd); getInput("lat", lat);
    getInput("thr", thr); getInput("yaw", yaw);
    // the commaand format is: BLIND,<fwd>,<lat>,<thr>,<yaw>
    std::string cmd = "BLIND," + std::to_string(fwd) + "," + std::to_string(lat) + "," + 
                      std::to_string(thr) + "," + std::to_string(yaw);
    set_controller_state(ros_state_, cmd);
    
    start_time_ = ros_state_->node->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BlindMove::onRunning() {
    if ((ros_state_->node->now() - start_time_).seconds() >= duration_) {
        set_controller_state(ros_state_, "IDLE");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void BlindMove::onHalted() { set_controller_state(ros_state_, "IDLE"); }