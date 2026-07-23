#include "conditions.hpp"
#include <cmath>

DetectImpact::DetectImpact(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::ConditionNode(name, config), ros_state_(ros_state) {}

BT::PortsList DetectImpact::providedPorts() {
    return { BT::InputPort<double>("threshold", 2.0, "Accel spike threshold") };
}

BT::NodeStatus DetectImpact::tick() {
    double threshold = 2.0; 
    getInput("threshold", threshold);
    
    double acc_x = ros_state_->telemetry.imu_xacc;
    double acc_y = ros_state_->telemetry.imu_yacc;
    double magnitude = std::sqrt(acc_x*acc_x + acc_y*acc_y);

    accel_history_.push_back(magnitude);
    if (accel_history_.size() > 10) accel_history_.pop_front();

    if (accel_history_.size() >= 2) {
        double diff = std::abs(accel_history_.back() - accel_history_.front());
        if (diff > threshold) {
            RCLCPP_WARN(ros_state_->node->get_logger(), "[IMPACT] Bump detected!");
            accel_history_.clear(); 
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::FAILURE;
}
//need to review this and verify if it works with the actual props