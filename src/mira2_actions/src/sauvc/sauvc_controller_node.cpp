#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <control_utils/control_utils.hpp>

#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <mutex>
#include <algorithm>
#include <cmath>

struct TargetData {
    rclcpp::Time last_seen;
    double norm_x = 0.5;
    double norm_y = 0.5;
    double area = 0.0;
};

class SauvcController : public rclcpp::Node {
public:
    SauvcController() : Node("sauvc_controller_node") {
        
        declare_parameter("sway_kp", 100.0);   declare_parameter("sway_kd", 20.0);
        declare_parameter("surge_kp", 100.0);  declare_parameter("surge_kd", 20.0);
        declare_parameter("heave_kp", 150.0);  declare_parameter("heave_kd", 30.0);
        declare_parameter("yaw_kp", 3.0);      declare_parameter("yaw_kd", 1.0);

        cmd_pub_ = this->create_publisher<custom_msgs::msg::Commands>("/master/commands", 10);
        
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/controller/state", 10, std::bind(&SauvcController::state_callback, this, std::placeholders::_1));
        
        vision_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/vision/detections", 10, std::bind(&SauvcController::vision_callback, this, std::placeholders::_1));
            
        telem_sub_ = this->create_subscription<custom_msgs::msg::Telemetry>(
            "/master/telemetry", 10, std::bind(&SauvcController::telemetry_callback, this, std::placeholders::_1));
    }

    void init() {
        sway_pid_  = std::make_unique<PID_Controller>("sway", shared_from_this());
        surge_pid_ = std::make_unique<PID_Controller>("surge", shared_from_this());
        heave_pid_ = std::make_unique<PID_Controller>("heave", shared_from_this());
        yaw_pid_   = std::make_unique<PID_Controller>("yaw", shared_from_this());

        sway_pid_->kp = get_parameter("sway_kp").as_double();   sway_pid_->kd = get_parameter("sway_kd").as_double();
        surge_pid_->kp = get_parameter("surge_kp").as_double(); surge_pid_->kd = get_parameter("surge_kd").as_double();
        heave_pid_->kp = get_parameter("heave_kp").as_double(); heave_pid_->kd = get_parameter("heave_kd").as_double();
        yaw_pid_->kp = get_parameter("yaw_kp").as_double();     yaw_pid_->kd = get_parameter("yaw_kd").as_double();

        // Base offsets must be 0 for our custom math below, since we add/subtract from 1500 manually
        sway_pid_->base_offset = 0; surge_pid_->base_offset = 0;
        heave_pid_->base_offset = 0; yaw_pid_->base_offset = 0;

        last_time_ = this->now();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SauvcController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "SAUVC C++ Muscle Controller Started at 50Hz. (STABILIZE ONLY)");
    }

private:
    std::unique_ptr<PID_Controller> sway_pid_;
    std::unique_ptr<PID_Controller> surge_pid_;
    std::unique_ptr<PID_Controller> heave_pid_;
    std::unique_ptr<PID_Controller> yaw_pid_;
    
    rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr vision_sub_;
    rclcpp::Subscription<custom_msgs::msg::Telemetry>::SharedPtr telem_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex data_mutex_;
    std::map<std::string, TargetData> targets_;
    
    double current_depth_ = 0.0;
    double current_heading_ = 0.0;
    double locked_heading_ = 0.0;

    std::string current_state_str_ = "IDLE";
    std::string prev_state_cmd_ = "IDLE";
    std::vector<std::string> state_args_;

    rclcpp::Time last_time_;

    std::vector<std::string> split(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(str);
        while (std::getline(tokenStream, token, delimiter)) tokens.push_back(token);
        return tokens;
    }

    void state_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg->data != prev_state_cmd_) {
            state_args_ = split(msg->data, ',');
            current_state_str_ = state_args_.empty() ? "IDLE" : state_args_[0];
            
            if (current_state_str_ == "TRACK") {
                locked_heading_ = current_heading_;
                RCLCPP_INFO(this->get_logger(), "Tracking started. Heading locked at %.1f deg", locked_heading_);
            }
            
            sway_pid_->emptyError(); surge_pid_->emptyError(); 
            heave_pid_->emptyError(); yaw_pid_->emptyError();
            
            prev_state_cmd_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "State changed to: %s", msg->data.c_str());
        }
    }

    void vision_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rclcpp::Time now = this->now();
        for (const auto& det : msg->detections) {
            double area = det.bbox.size_x * det.bbox.size_y;
            targets_[det.id] = {now, det.bbox.center.position.x, det.bbox.center.position.y, area};
        }
    }

    void telemetry_callback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_depth_ = msg->external_pressure;
        current_heading_ = msg->heading;
    }

    bool get_target(const std::string& id, TargetData& data) {
        auto it = targets_.find(id);
        if (it != targets_.end() && (this->now() - it->second.last_seen).seconds() < 1.0) {
            data = it->second;
            return true;
        }
        return false;
    }

    void control_loop() {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        std::lock_guard<std::mutex> lock(data_mutex_);
        custom_msgs::msg::Commands cmd;
        cmd.arm = true; 
        
        // EVERYTHING ALWAYS RUNS IN STABILIZE!
        cmd.mode = "STABILIZE"; 
        
        float fwd_pwm = 1500, lat_pwm = 1500, thr_pwm = 1500, yaw_pwm = 1500;
        cmd.pitch = 1500; cmd.roll = 1500; cmd.servo1 = 1500; cmd.servo2 = 1500;

        if (current_state_str_ == "DIVE") {
            if (state_args_.size() >= 2) {
                double target_depth = std::stod(state_args_[1]);
                double err = target_depth - current_depth_;
                
                // If error is positive (target > current), we need to go DOWN.
                // PWM < 1500 goes down. So we SUBTRACT the PID output.
                thr_pwm = 1500 - heave_pid_->pid_control(err, dt, false);
            }
        } 
        else if (current_state_str_ == "SEARCH") {
            yaw_pwm = 1560; // Gentle spin
        }
        else if (current_state_str_ == "TRACK") {
            if (state_args_.size() >= 4) {
                std::string target = state_args_[1];
                std::string cam_mode = state_args_[2];
                std::string avoid = state_args_[3];

                TargetData t_data;
                if (get_target(target, t_data)) {
                    
                    // Sway (X error)
                    double err_x = t_data.norm_x - 0.5;
                    lat_pwm = 1500 + sway_pid_->pid_control(err_x, dt, false);

                    // Obstacle Avoidance Override
                    TargetData avoid_data;
                    if (avoid != "none" && avoid != "" && get_target(avoid, avoid_data)) {
                        if (avoid_data.area > 0.05) {
                            lat_pwm = (avoid_data.norm_x > 0.5) ? 1350 : 1650; // Push hard away
                        }
                    }

                    if (cam_mode == "front_cam") {
                        fwd_pwm = 1600; // Constant approach speed
                    } 
                    else if (cam_mode == "bottom_cam") {
                        // Forward/Backward driven by Y-axis in bottom cam
                        double err_y = t_data.norm_y - 0.5;
                        fwd_pwm = 1500 - surge_pid_->pid_control(err_y, dt, false);
                        
                        // Heave (Depth) driven by bounding box Area
                        double target_area = 0.5; // We want it to fill 50% of screen
                        double depth_err = target_area - t_data.area; 
                        // If err is positive (too small), we need to sink (<1500)
                        thr_pwm = 1500 - heave_pid_->pid_control(depth_err, dt, false);
                    }

                    // Yaw Heading Lock
                    double yaw_err = locked_heading_ - current_heading_;
                    while (yaw_err > 180.0) yaw_err -= 360.0;
                    while (yaw_err < -180.0) yaw_err += 360.0;
                    yaw_pwm = 1500 + yaw_pid_->pid_control(yaw_err, dt, false);
                } 
            }
        }
        else if (current_state_str_ == "BLIND") {
            if (state_args_.size() >= 5) {
                fwd_pwm = std::stoi(state_args_[1]);
                lat_pwm = std::stoi(state_args_[2]);
                thr_pwm = std::stoi(state_args_[3]);
                yaw_pwm = std::stoi(state_args_[4]);
            }
        }

        // Apply and clamp
        cmd.lateral = std::clamp((int)lat_pwm, 1100, 1900);
        cmd.forward = std::clamp((int)fwd_pwm, 1100, 1900);
        cmd.yaw     = std::clamp((int)yaw_pwm, 1100, 1900);
        cmd.thrust  = std::clamp((int)thr_pwm, 1100, 1900);

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SauvcController>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}