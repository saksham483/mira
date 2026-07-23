#pragma once

#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <map>
#include <mutex>
#include <string>

struct ROSState {
    custom_msgs::msg::Telemetry telemetry;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr cmd_publisher;
    
private:
    rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr telemetry_sub;

public:
    ROSState(rclcpp::Node::SharedPtr n) : node(n) {
        cmd_publisher = node->create_publisher<custom_msgs::msg::Commands>("/master/commands", 10);
        telemetry_sub = node->create_subscription<custom_msgs::msg::Telemetry>(
            "/master/telemetry", 10,
            [this](const custom_msgs::msg::Telemetry::SharedPtr msg) {
                telemetry = *msg;
            });
    }
};

// Helper function to safely stop motors
void publish_neutral(ROSState* ros_state, std::string mode = "ALT_HOLD");

class VisionTracker {
public:
    VisionTracker(ROSState* ros_state);
    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    bool isVisible(const std::string& target, double timeout_sec);
    bool getTargetData(const std::string& target, double& norm_x, double& norm_y, double& area);
private:
    ROSState* ros_state_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
    std::mutex vision_mutex_;
    struct TargetData {
        rclcpp::Time last_seen;
        double norm_x, norm_y, area;
    };
    std::map<std::string, TargetData> targets_;
};