#include "conditions.hpp"

IsTaskEnabled::IsTaskEnabled(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList IsTaskEnabled::providedPorts() {
    return { BT::InputPort<bool>("enabled", true, "True to run task") };
}

BT::NodeStatus IsTaskEnabled::tick() {
    bool enabled = true;
    getInput("enabled", enabled);
    return enabled ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
//this is a condition node made for just activating and dectivating the task nodes