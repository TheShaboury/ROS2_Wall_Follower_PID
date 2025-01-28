#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>

class WallFollowNode : public rclcpp::Node {
public:
    WallFollowNode() : Node("wall_follower_controller") {
        speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle_command", 10);
        steering_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/steering_command", 10);
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/autodrive/f1tenth_1/lidar",
            10,
            std::bind(&WallFollowNode::scan_callback, this, std::placeholders::_1));
        points_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/autodrive/f1tenth_1/ips",
            10,
            std::bind(&WallFollowNode::callback_points, this, std::placeholders::_1));

        // Initialize parameters
        desired_wall_distance_ = 1.3;
        max_steering_angle_ = 1.0;
        max_speed_ = 0.15;
        min_speed_ = 0.1;
        safe_distance_ = 2.5;
        stop_distance_ = 1.4;
        steering_sensitivity_ = 3.9;
        point_dict_ = {0.0, 0.0, 0.0};
        kp_ = 1.0;
        ki_ = 0.2;
        kd_ = 0.06;
        integral_ = 0.0;
        previous_error_ = 0.0;
        last_callback_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "WallFollowNode has been started.");
    }

private:
    // Callback for receiving IPS (Indoor Positioning System) data
    void callback_points(const geometry_msgs::msg::Point::SharedPtr point) {
        RCLCPP_INFO(this->get_logger(), "Point: x=%f, y=%f, z=%f",
                    point->x, point->y, point->z);
        point_dict_ = {point->x, point->y, point->z};
    }

    // Main control loop callback
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto current_time = this->now();
        double dt = (current_time - last_callback_time_).seconds();
        last_callback_time_ = current_time;

        // Process LIDAR data
        double front_range = std::numeric_limits<double>::max();
        double left_range = std::numeric_limits<double>::max();
        double right_range = std::numeric_limits<double>::max();

        // Find minimum ranges in each sector
        for (size_t i = 520; i < 600; ++i) {
            if (i < msg->ranges.size()) {
                front_range = std::min(front_range, static_cast<double>(msg->ranges[i]));
            }
        }
        for (size_t i = 420; i < 480; ++i) {
            if (i < msg->ranges.size()) {
                left_range = std::min(left_range, static_cast<double>(msg->ranges[i]));
            }
        }
        for (size_t i = 600; i < 660; ++i) {
            if (i < msg->ranges.size()) {
                right_range = std::min(right_range, static_cast<double>(msg->ranges[i]));
            }
        }

        // Calculate error for PID control
        double error;
        if (left_range < right_range) {
            error = desired_wall_distance_ - left_range;
        } else {
            error = right_range - desired_wall_distance_;
        }

        // PID control
        integral_ += error * dt;
        integral_ = std::max(std::min(integral_, 1.0), -1.0);
        double derivative = (error - previous_error_) / dt;
        double steering_angle = kp_ * error + ki_ * integral_ + kd_ * derivative;
        previous_error_ = error;

        // Limit steering angle
        steering_angle = std::max(std::min(steering_angle, max_steering_angle_), -max_steering_angle_);
        
        // Calculate speed based on front distance and steering angle
        double speed = calculate_speed(front_range, std::abs(steering_angle));
        
        // Publish control commands
        publish_commands(speed, steering_angle);
    }

    // Calculate speed based on front distance and steering angle
    double calculate_speed(double front_distance, double steering_angle) {
        double distance_factor = (front_distance - stop_distance_) / (safe_distance_ - stop_distance_);
        distance_factor = std::max(std::min(distance_factor, 1.0), 0.0);
        double distance_speed = min_speed_ + (max_speed_ - min_speed_) * distance_factor;

        double angle_factor = 1.0 - std::pow(std::abs(steering_angle) / max_steering_angle_, steering_sensitivity_);
        angle_factor = std::max(std::min(angle_factor, 1.0), 0.0);
        double angle_speed = min_speed_ + (max_speed_ - min_speed_) * angle_factor;

        double speed = std::min(distance_speed, angle_speed);
        return std::max(std::min(speed, max_speed_), min_speed_);
    }

    // Publish speed and steering commands
    void publish_commands(double speed, double steering_angle) {
        auto speed_msg = std_msgs::msg::Float32();
        speed_msg.data = static_cast<float>(speed);
        speed_publisher_->publish(speed_msg);

        auto steering_msg = std_msgs::msg::Float32();
        steering_msg.data = static_cast<float>(steering_angle);
        steering_publisher_->publish(steering_msg);

        RCLCPP_INFO(this->get_logger(), "Published commands: speed=%.2f, steering_angle=%.2f°",
                    speed, steering_angle * 180.0 / M_PI);
    }

    // ROS publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr points_subscription_;

    // Controller parameters
    double desired_wall_distance_;
    double max_steering_angle_;
    double max_speed_;
    double min_speed_;
    double safe_distance_;
    double stop_distance_;
    double steering_sensitivity_;
    struct {
        double x, y, z;
    } point_dict_;
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double previous_error_;
    rclcpp::Time last_callback_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
