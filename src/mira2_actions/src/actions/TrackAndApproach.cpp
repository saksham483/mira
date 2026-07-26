#include "actions.hpp"

TrackAndApproach::TrackAndApproach(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state, std::shared_ptr<VisionTracker> tracker)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state), tracker_(tracker) {}

BT::PortsList TrackAndApproach::providedPorts() {
    return {
        BT::InputPort<std::string>("target", "Name of object to track"),
        BT::InputPort<std::string>("avoid", "none", "Name of obstacle to avoid"),
        BT::InputPort<std::string>("mode", "front_cam", "front_cam or bottom_cam"),
        BT::InputPort<double>("success_area", 0.6, "Area ratio to consider successful"),
        BT::InputPort<double>("lost_timeout", 3.0, "Time before failing if target lost")
    };
}

BT::NodeStatus TrackAndApproach::onStart() {
    getInput("target", target_object_);
    getInput("avoid", avoid_object_);
    getInput("mode", mode_);
    getInput("success_area", success_area_);
    getInput("lost_timeout", lost_timeout_);
    std::string cmd = "TRACK," + target_object_ + "," + mode_ + "," + avoid_object_;
    set_controller_state(ros_state_, cmd);
    
    start_time_ = ros_state_->node->now();
    last_seen_time_ = start_time_;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackAndApproach::onRunning() {
    rclcpp::Time now = ros_state_->node->now();

    if (tracker_->isVisible(target_object_, 0.2)) {
        last_seen_time_ = now;
    }

    if ((now - last_seen_time_).seconds() > lost_timeout_) {
        RCLCPP_WARN(ros_state_->node->get_logger(), "[TRACK] Lost target: %s", target_object_.c_str());
        set_controller_state(ros_state_, "IDLE");
        return BT::NodeStatus::FAILURE;
    }

    double norm_x, norm_y, area;
    if (tracker_->getTargetData(target_object_, norm_x, norm_y, area)) {
        if (area >= success_area_) {
            RCLCPP_INFO(ros_state_->node->get_logger(), "[TRACK] Arrived at %s!", target_object_.c_str());
            set_controller_state(ros_state_, "IDLE");
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void TrackAndApproach::onHalted() { set_controller_state(ros_state_, "IDLE"); }