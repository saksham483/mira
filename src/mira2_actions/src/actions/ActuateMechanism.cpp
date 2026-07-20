#include "actions.hpp"

ActuateMechanism::ActuateMechanism(const std::string& name, const BT::NodeConfiguration& config, ROSState* ros_state)
    : BT::SyncActionNode(name, config), ros_state_(ros_state) {}

BT::PortsList ActuateMechanism::providedPorts() {
    return {
        BT::InputPort<int>("servo_id", 1, "1 or 2"),
        BT::InputPort<int>("pwm", 1900, "PWM to send to servo")
    };
}

BT::NodeStatus ActuateMechanism::tick() {
    int servo_id = 1; getInput("servo_id", servo_id);
    int pwm = 1900; getInput("pwm", pwm);

    custom_msgs::msg::Commands cmd;
    cmd.arm = true; 
    cmd.mode = "ALT_HOLD";
    cmd.forward = 1500; 
    cmd.lateral = 1500; 
    cmd.thrust = 1500; 
    cmd.yaw = 1500;
    cmd.pitch = 1500;
    cmd.roll = 1500;
    cmd.servo1 = (servo_id == 1) ? pwm : 1500;
    cmd.servo2 = (servo_id == 2) ? pwm : 1500;

    ros_state_->cmd_publisher->publish(cmd);
    RCLCPP_INFO(ros_state_->node->get_logger(), "[MECHANISM] Servo %d actuated to %d PWM!", servo_id, pwm);
    
    return BT::NodeStatus::SUCCESS;
}