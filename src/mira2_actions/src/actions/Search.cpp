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
    RCLCPP_INFO(ros_state_->node->get_logger(), "[SEARCH] Looking for %s", target_object_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Search::onRunning() {
    if ((ros_state_->node->now() - start_time_).seconds() > timeout_) {
        RCLCPP_WARN(ros_state_->node->get_logger(), "[SEARCH] Timeout finding %s", target_object_.c_str());
        publish_neutral(ros_state_);
        return BT::NodeStatus::FAILURE;
    }

    if (tracker_->isVisible(target_object_, 0.5)) {
        RCLCPP_INFO(ros_state_->node->get_logger(), "[SEARCH] Found %s!", target_object_.c_str());
        publish_neutral(ros_state_);
        return BT::NodeStatus::SUCCESS;
    }

    custom_msgs::msg::Commands cmd;
    cmd.arm = true; 
    cmd.mode = "ALT_HOLD";
    cmd.forward = 1500; 
    cmd.lateral = 1500; 
    cmd.thrust = 1500;
    cmd.yaw = 1550; //slow yaw rotation to right
    cmd.pitch = 1500; 
    cmd.roll = 1500;
    cmd.servo1 = 1500; 
    cmd.servo2 = 1500;
    ros_state_->cmd_publisher->publish(cmd);

    return BT::NodeStatus::RUNNING;
}

void Search::onHalted() {
    publish_neutral(ros_state_);
}