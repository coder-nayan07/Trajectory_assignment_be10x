#include <chrono>
#include <cmath>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

struct Point2D {
    double x, y;
};

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("planner_node") {
        // Create a publisher that outputs the path to the topic "smooth_path"
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("smooth_path", 10);
        
        // Define the "Discrete Waypoints" (Task 1)
        // We use a simple square loop for testing
        waypoints_ = {
            {0.0, 0.0},
            {2.0, 0.0},
            {2.0, 2.0},
            {0.0, 2.0},
            {0.0, 0.0}
        };

        // Create a timer to publish the path every 1 second
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&PlannerNode::publish_path, this));
            
        RCLCPP_INFO(this->get_logger(), "Planner Node Initialized");
    }

private:
    std::vector<Point2D> waypoints_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Catmull-Rom Spline Math (Task 1: Smoothing Algorithm) ---
    // This function calculates a point between p1 and p2, influenced by p0 and p3
    Point2D catmull_rom(double t, Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
        double t2 = t * t;
        double t3 = t2 * t;

        // Matrix calculation for Catmull-Rom
        double x = 0.5 * ((2.0 * p1.x) + 
                          (-p0.x + p2.x) * t + 
                          (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 + 
                          (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);

        double y = 0.5 * ((2.0 * p1.y) + 
                          (-p0.y + p2.y) * t + 
                          (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 + 
                          (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);
                          
        return {x, y};
    }

    void publish_path() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "odom"; // The path is in the Odometry frame

        if (waypoints_.size() < 2) return;

        // Add padding points (duplicate start and end) so the spline works at boundaries
        std::vector<Point2D> padded_wp = waypoints_;
        padded_wp.insert(padded_wp.begin(), waypoints_.front()); 
        padded_wp.push_back(waypoints_.back()); 

        // Loop through segments
        for (size_t i = 0; i < padded_wp.size() - 3; ++i) {
            Point2D p0 = padded_wp[i];
            Point2D p1 = padded_wp[i+1];
            Point2D p2 = padded_wp[i+2];
            Point2D p3 = padded_wp[i+3];

            // Generate 20 points per segment (Task 2: Sampling)
            int steps = 20; 
            for (int s = 0; s < steps; s++) {
                double t = (double)s / (double)steps;
                Point2D smooth_p = catmull_rom(t, p0, p1, p2, p3);

                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                pose.pose.position.x = smooth_p.x;
                pose.pose.position.y = smooth_p.y;
                pose.pose.orientation.w = 1.0; // Identity orientation
                
                path_msg.poses.push_back(pose);
            }
        }

        path_pub_->publish(path_msg);
        RCLCPP_INFO_ONCE(this->get_logger(), "Path Generated & Published with %zu points", path_msg.poses.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
