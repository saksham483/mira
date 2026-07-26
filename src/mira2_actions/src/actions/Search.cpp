#include "actions.hpp"
//here the search motion is yawing right will need to find optimal motion for it 
Search::Search(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state, std::shared_ptr<VisionTracker> tracker)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state), tracker_(tracker) {}

BT::PortsList Search::providedPorts() {
    return { 
        BT::InputPort<std::string>("target", "Target to search for"),
        BT::InputPort<double>("timeout", 30.0, "Search timeout") 
    };
}

BT::NodeStatus Search::onStart() {
    getInput("target", target_object_);
    getInput("timeout", timeout_);
    start_time_ = ros_state_->node->now();
    set_controller_state(ros_state_, "SEARCH," + target_object_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Search::onRunning() {
    if ((ros_state_->node->now() - start_time_).seconds() > timeout_) {
        set_controller_state(ros_state_, "IDLE");
        return BT::NodeStatus::FAILURE;
    }

    if (tracker_->isVisible(target_object_, 0.5)) {
        set_controller_state(ros_state_, "IDLE");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void Search::onHalted() { set_controller_state(ros_state_, "IDLE"); }