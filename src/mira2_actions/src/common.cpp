#include "common.hpp"

void publish_neutral(ROSState* ros_state, std::string mode) {
    custom_msgs::msg::Commands cmd;
    cmd.arm = true; 
    cmd.mode = mode;
    cmd.forward = 1500; cmd.lateral = 1500;
    cmd.thrust = 1500;  cmd.yaw = 1500;
    cmd.pitch = 1500;   cmd.roll = 1500;
    cmd.servo1 = 1500;  cmd.servo2 = 1500;
    ros_state->cmd_publisher->publish(cmd);
}

VisionTracker::VisionTracker(ROSState* ros_state) : ros_state_(ros_state) {
    sub_ = ros_state_->node->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/vision/detections", 10,
        std::bind(&VisionTracker::detection_callback, this, std::placeholders::_1));
}

void VisionTracker::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    rclcpp::Time now = ros_state_->node->now();
    for (const auto& det : msg->detections) {
        double area = det.bbox.size_x * det.bbox.size_y;
        targets_[det.id] = {now, det.bbox.center.position.x, det.bbox.center.position.y, area};
    }
}

bool VisionTracker::isVisible(const std::string& target, double timeout_sec) {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    if (targets_.find(target) == targets_.end()) return false;
    double age = (ros_state_->node->now() - targets_[target].last_seen).seconds();
    return age <= timeout_sec;
}

bool VisionTracker::getTargetData(const std::string& target, double& norm_x, double& norm_y, double& area) {
    std::lock_guard<std::mutex> lock(vision_mutex_);
    if (targets_.find(target) == targets_.end()) return false;
    norm_x = targets_[target].norm_x;
    norm_y = targets_[target].norm_y;
    area = targets_[target].area;
    return true;
}