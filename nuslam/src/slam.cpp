#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "armadillo"

class SLAM : public rclcpp::Node {
    public:
        SLAM() : Node("slam") {
            odom_sub_ = declare_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SLAM::odom_callback, this, std::placeholders::_1));
            sensor_marker_sub_ = declare_subscription<visualization_msgs::msg::MarkerArray>("sensor_data", 10, std::bind(&SLAM::sensor_data_callback, this, std::placeholders::_1));
        }

        auto rate = 10.0;
        auto dt = 1.0 / rate;
        timer_ = this->create_wall_timer(dt, std::bind(&SLAM::timer_callback, this));
    private:
        // robot pose
        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;

        // marker list
        visualization_msgs::msg::Marker sensor_markers_[10];

        // implement kalman filter
        void timer_callback() {
            
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            x_ = msg->pose.pose.position.x;
            y_ = msg->pose.pose.position.y;
            theta_ = msg->pose.pose.orientation.w;
        }

        void sensor_data_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
            for (int i = 0; i < 10; i++) {
                if (msg->markers[i].action == visualization_msgs::msg::Marker::ADD) {
                    sensor_markers_[i] = msg->markers[i];
                } else if (msg->markers[i].action == visualization_msgs::msg::Marker::DELETE) {
                    sensor_markers_[i] = visualization_msgs::msg::Marker();
                }
            }
        }
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_marker_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SLAM>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
