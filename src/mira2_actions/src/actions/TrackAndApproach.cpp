#include "actions.hpp"
#include <algorithm>

TrackAndApproach::TrackAndApproach(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state, std::shared_ptr<VisionTracker> tracker)
    : BT::StatefulActionNode(name, config), ros_state_(ros_state), tracker_(tracker),
      lateral_pid_(name+"_lat", ros_state->node), forward_pid_(name+"_fwd", ros_state->node), yaw_pid_(name+"_yaw", ros_state->node) {}

BT::PortsList TrackAndApproach::providedPorts() {
    return {
        BT::InputPort<std::string>("target", "Name of object to track"),
        BT::InputPort<std::string>("avoid", "", "Name of obstacle to avoid"),
        BT::InputPort<std::string>("mode", "forward", "forward or overhead"),
        BT::InputPort<double>("success_area", 0.6, "Area ratio to succeed"),
        BT::InputPort<double>("lost_timeout", 3.0, "Time before failing if lost"),
        BT::InputPort<double>("lat_kp", 100.0, "Lateral Kp"),
        BT::InputPort<double>("fwd_kp", 100.0, "Forward Kp"),
        BT::InputPort<double>("yaw_kp", 2.0, "Yaw Kp")
    };
}

BT::NodeStatus TrackAndApproach::onStart() {
    getInput("target", target_object_); getInput("avoid", avoid_object_);
    getInput("mode", mode_); getInput("success_area", success_area_);
    getInput("lost_timeout", lost_timeout_);
    
    lateral_pid_.kp = getInput<double>("lat_kp").value(); lateral_pid_.base_offset = 1500; lateral_pid_.emptyError();
    forward_pid_.kp = getInput<double>("fwd_kp").value(); forward_pid_.base_offset = 1500; forward_pid_.emptyError();
    yaw_pid_.kp = getInput<double>("yaw_kp").value(); yaw_pid_.base_offset = 1500; yaw_pid_.emptyError();

    locked_heading_ = ros_state_->telemetry.heading;
    start_time_ = ros_state_->node->now();
    last_time_ = start_time_;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackAndApproach::onRunning() {
    rclcpp::Time now = ros_state_->node->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (!tracker_->isVisible(target_object_, lost_timeout_)) {
        RCLCPP_WARN(ros_state_->node->get_logger(), "[TRACK] Lost target: %s", target_object_.c_str());
        publish_neutral(ros_state_);
        return BT::NodeStatus::FAILURE;
    }

    double norm_x, norm_y, area;
    tracker_->getTargetData(target_object_, norm_x, norm_y, area);

    if (area >= success_area_) {
        RCLCPP_INFO(ros_state_->node->get_logger(), "[TRACK] Approach complete: %s", target_object_.c_str());
        publish_neutral(ros_state_);
        return BT::NodeStatus::SUCCESS;
    }

    double err_x = norm_x - 0.5; 
    float lat_pwm = lateral_pid_.pid_control(err_x, dt, false);
    
    if (!avoid_object_.empty() && tracker_->isVisible(avoid_object_, 0.5)) {
        double o_x, o_y, o_area;
        tracker_->getTargetData(avoid_object_, o_x, o_y, o_area);
        if (o_area > 0.05) { 
            lat_pwm = (o_x > 0.5) ? 1400 : 1600; // Evade
            RCLCPP_WARN_THROTTLE(ros_state_->node->get_logger(), *ros_state_->node->get_clock(), 1000, 
                "[AVOID] Evading %s!", avoid_object_.c_str());
        }
    }

    float fwd_pwm = 1500;
    if (mode_ == "forward") fwd_pwm = 1600; 
    else if (mode_ == "overhead") fwd_pwm = forward_pid_.pid_control(norm_y - 0.5, dt, false);

    double yaw_err = locked_heading_ - ros_state_->telemetry.heading;
    while (yaw_err > 180.0) yaw_err -= 360.0;
    while (yaw_err < -180.0) yaw_err += 360.0;
    float yaw_pwm = yaw_pid_.pid_control(yaw_err, dt, false);

    custom_msgs::msg::Commands cmd;
    cmd.arm = true; cmd.mode = "ALT_HOLD";
    cmd.lateral = std::clamp((int)lat_pwm, 1100, 1900);
    cmd.forward = std::clamp((int)fwd_pwm, 1100, 1900);
    cmd.yaw = std::clamp((int)yaw_pwm, 1100, 1900);
    cmd.thrust = 1500;
    ros_state_->cmd_publisher->publish(cmd);

    return BT::NodeStatus::RUNNING;
}

void TrackAndApproach::onHalted() {
    publish_neutral(ros_state_);
    lateral_pid_.emptyError(); forward_pid_.emptyError(); yaw_pid_.emptyError();
}