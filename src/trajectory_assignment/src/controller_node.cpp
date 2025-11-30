#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "smooth_path", 10, std::bind(&ControllerNode::path_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(), std::bind(&ControllerNode::odom_callback, this, _1));

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Controller Node Initialized. Waiting for path...");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    std::vector<geometry_msgs::msg::PoseStamped> path_points_;
    bool path_received_ = false;
    int last_closest_idx_ = 0; // Keep track of progress

    // Tuning Parameters
    double lookahead_dist_ = 0.4; 
    double velocity_ = 0.2;       

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        // Reset progress if a new path comes in
        if (msg->poses.size() != path_points_.size()) {
             last_closest_idx_ = 0;
        }
        path_points_ = msg->poses;
        path_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "Path Received with %zu points", path_points_.size());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!path_received_ || path_points_.empty()) return;

        double rx = msg->pose.pose.position.x;
        double ry = msg->pose.pose.position.y;
        
        // Get Yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 1. Find Closest Point (But only look forward from where we were)
        // We limit search to next 10 points to enforce sequential following
        int search_limit = std::min((int)path_points_.size(), last_closest_idx_ + 10);
        
        int closest_idx = last_closest_idx_;
        double min_dist = 10000.0;
        
        for (int i = last_closest_idx_; i < search_limit; i++) {
            double px = path_points_[i].pose.position.x;
            double py = path_points_[i].pose.position.y;
            double d = std::hypot(px - rx, py - ry);
            if (d < min_dist) {
                min_dist = d;
                closest_idx = i;
            }
        }
        
        // Update our progress tracker
        last_closest_idx_ = closest_idx;

        // 2. Check for Completion
        // Only stop if we are at the very last point
        if (closest_idx >= (int)path_points_.size() - 2) {
             geometry_msgs::msg::Twist stop_cmd;
             vel_pub_->publish(stop_cmd);
             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached!");
             return;
        }

        // 3. Find Lookahead Point
        double target_x = rx; 
        double target_y = ry;
        
        for (size_t i = closest_idx; i < path_points_.size(); i++) {
            double px = path_points_[i].pose.position.x;
            double py = path_points_[i].pose.position.y;
            double d = std::hypot(px - rx, py - ry);
            
            if (d > lookahead_dist_) {
                target_x = px;
                target_y = py;
                break;
            }
        }

        // 4. Calculate Steering
        double angle_to_target = std::atan2(target_y - ry, target_x - rx);
        double error = angle_to_target - yaw;

        while (error > M_PI) error -= 2.0 * M_PI;
        while (error < -M_PI) error += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = velocity_;
        cmd.angular.z = 1.5 * error; 

        vel_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
