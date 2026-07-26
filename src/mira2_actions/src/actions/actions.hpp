#pragma once
#include <behaviortree_cpp/action_node.h>
#include "../common.hpp"

class DiveToDepth : public BT::StatefulActionNode {
public:
    DiveToDepth(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ROSState* ros_state_;
    double target_depth_, tolerance_;
};

class Search : public BT::StatefulActionNode {
public:
    Search(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state, std::shared_ptr<VisionTracker> tracker);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ROSState* ros_state_;
    std::shared_ptr<VisionTracker> tracker_;
    std::string target_object_;
    double timeout_;
    rclcpp::Time start_time_;
};

class TrackAndApproach : public BT::StatefulActionNode {
public:
    TrackAndApproach(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state, std::shared_ptr<VisionTracker> tracker);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ROSState* ros_state_;
    std::shared_ptr<VisionTracker> tracker_;
    std::string target_object_, avoid_object_, mode_;
    double success_area_, lost_timeout_;
    rclcpp::Time start_time_, last_seen_time_;
};

class BlindMove : public BT::StatefulActionNode {
public:
    BlindMove(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ROSState* ros_state_;
    double duration_;
    rclcpp::Time start_time_;
};
//class meant to control the robotic arm will work on it when we have a arm
class ActuateMechanism : public BT::SyncActionNode {
public:
    ActuateMechanism(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    ROSState* ros_state_;
};

// it is a placeholder for now will implement it afterwards
class EmergencySurface : public BT::StatefulActionNode {
public:
    EmergencySurface(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ROSState* ros_state_;
};

class SetFlightMode : public BT::StatefulActionNode {
public:
    SetFlightMode(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ROSState* ros_state_;
    std::string target_mode_;
    rclcpp::Time start_time_;
};