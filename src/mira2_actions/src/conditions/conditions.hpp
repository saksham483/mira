#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <deque>
#include "../common.hpp"

class IsEmergency : public BT::ConditionNode {
public:
    IsEmergency(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    ROSState* ros_state_;
};

class IsTaskEnabled : public BT::ConditionNode {
public:
    IsTaskEnabled(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class DetectImpact : public BT::ConditionNode {
public:
    DetectImpact(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    ROSState* ros_state_;
    std::deque<double> accel_history_;
};