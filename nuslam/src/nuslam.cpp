#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ekf.hpp"


class SLAM : public rclcpp::Node {
    public:
        SLAM() : Node("nuslam") {
            odom_sub_ = declare_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SLAM::odom_callback, this, std::placeholders::_1));
            sensor_marker_sub_ = declare_subscription<visualization_msgs::msg::MarkerArray>("sensor_data", 10, std::bind(&SLAM::sensor_data_callback, this, std::placeholders::_1));
        }

    private:
        // robot pose
        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;

        // previous robot pose        
        double prev_x_ = 0.0;
        double prev_y_ = 0.0;
        double prev_theta_ = 0.0;

        // maximum number of landmarks
        int obs_num = 10;

        // initialize extended kalman filter
        EKF ekf_ = EKF(obs_num);

        // marker list
        visualization_msgs::msg::Marker sensor_markers_[10];


        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            x_ = msg->pose.pose.position.x;
            y_ = msg->pose.pose.position.y;
            theta_ = 2.0 * std::asin(msg->pose.pose.orientation.z);

            //predict
            std::vector<double> action = {theta_ - prev_theta_, std::hypot(x_ - prev_x_, y_ - prev_y_)};
            ekf_.predict(action);
            prev_x_ = x_;
            prev_y_ = y_;
            prev_theta_ = theta_;
        }

        void sensor_data_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
            for (int i = 0; i < msg->markers.size(); i++) {
                if (msg->markers[i].action == visualization_msgs::msg::Marker::ADD) {
                    sensor_markers_[i] = msg->markers[i];

                    //update
                    double obs_x = msg->markers[i].pose.position.x;
                    double obs_y = msg->markers[i].pose.position.y;
                    double dist_obs = std::hypot(obs_x, obs_y);
                    double angle_obs = std::atan2(obs_y, obs_x);
                    ekf_.update(i, dist_obs, angle_obs);
                }
                else if (msg->markers[i].action == visualization_msgs::msg::Marker::DELETE) {
                    sensor_markers_[i] = visualization_msgs::msg::Marker();
                }
            }
        }
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_marker_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SLAM>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
