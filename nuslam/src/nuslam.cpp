/// \file
/// \brief A ROS2 node that implements SLAM using an Extended Kalman Filter
///
/// PUBLISHES:
///     slam_path (nav_msgs/msg/Path): the SLAM-estimated path of the robot
///     odom_measurement (nav_msgs/msg/Path): the uncorrected odometry path
/// SUBSCRIBES:
///     odom (nav_msgs/msg/Odometry): the odometry of the robot
///     sensor_data (visualization_msgs/msg/MarkerArray): the detected landmarks

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlelib/se2d.hpp"
#undef pi  // angle.hpp defines pi as a macro which conflicts with arma::Datum<T>::pi
#include "ekf.hpp"


class SLAM : public rclcpp::Node {
    public:
        /// \brief Constructor for SLAM
        SLAM() : Node("nuslam") {
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SLAM::odom_callback, this, std::placeholders::_1));
            sensor_marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>("sensor_data", 10, std::bind(&SLAM::sensor_data_callback, this, std::placeholders::_1));
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            path_pub_ = create_publisher<nav_msgs::msg::Path>("slam_path", 10);
            slam_path_.header.frame_id = "map";
            odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("odom_measurement", 10);
            odom_path_.header.frame_id = "map";
            marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("slam_markers", 10);
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
        std::vector<visualization_msgs::msg::Marker> sensor_markers_ = std::vector<visualization_msgs::msg::Marker>(10);

        /// \brief Callback function for odometry
        /// \param msg Odometry message
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            x_ = msg->pose.pose.position.x;
            y_ = msg->pose.pose.position.y;
            theta_ = 2.0 * std::asin(msg->pose.pose.orientation.z);

            //predict
            double dx = x_ - prev_x_;
            double dy = y_ - prev_y_;
            double signed_dist = dx * std::cos(prev_theta_) + dy * std::sin(prev_theta_);
            std::vector<double> action = {theta_ - prev_theta_, signed_dist};
            ekf_.predict(action);
            prev_x_ = x_;
            prev_y_ = y_;
            prev_theta_ = theta_;

            auto now = this->now();

            // SLAM estimated path
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = now;
            pose.header.frame_id = "map";
            pose.pose.position.x = ekf_.get_x();
            pose.pose.position.y = ekf_.get_y();
            pose.pose.orientation.z = std::sin(ekf_.get_theta() / 2.0);
            pose.pose.orientation.w = std::cos(ekf_.get_theta() / 2.0);
            slam_path_.header.stamp = now;
            slam_path_.poses.push_back(pose);
            path_pub_->publish(slam_path_);

            // Odometry (uncorrected) path
            geometry_msgs::msg::PoseStamped odom_pose;
            odom_pose.header.stamp = now;
            odom_pose.header.frame_id = "map";
            odom_pose.pose.position.x = x_;
            odom_pose.pose.position.y = y_;
            odom_pose.pose.orientation.z = std::sin(theta_ / 2.0);
            odom_pose.pose.orientation.w = std::cos(theta_ / 2.0);
            odom_path_.header.stamp = now;
            odom_path_.poses.push_back(odom_pose);
            odom_path_pub_->publish(odom_path_);

            publish_transforms();
        }

        /// \brief Callback function for sensor data
        /// \param msg Sensor data message
        void sensor_data_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
            for (int i = 0; i < int(msg->markers.size()); i++) {
                if (msg->markers.at(i).action == visualization_msgs::msg::Marker::ADD) {
                    sensor_markers_.at(i) = msg->markers.at(i);

                    //update
                    double obs_x = msg->markers.at(i).pose.position.x;
                    double obs_y = msg->markers.at(i).pose.position.y;
                    double dist_obs = std::hypot(obs_x, obs_y);
                    double angle_obs = std::atan2(obs_y, obs_x);
                    // ekf_.update(msg->markers.at(i).id, dist_obs, angle_obs);
                    ekf_.update_with_landmark(dist_obs, angle_obs);
                }
                else if (msg->markers.at(i).action == visualization_msgs::msg::Marker::DELETE) {
                    sensor_markers_.at(i) = visualization_msgs::msg::Marker();
                }
            }
            publish_transforms();
            slam_markers_.markers.clear();
            for (int i = 0; i < ekf_.get_N(); i++) {
                visualization_msgs::msg::Marker marker;
                marker.header.stamp = this->now();
                marker.header.frame_id = "map";
                marker.ns = "slam_markers";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = ekf_.get_obs_x(i);
                marker.pose.position.y = ekf_.get_obs_y(i);
                marker.pose.position.z = 0.1;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.2;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                slam_markers_.markers.push_back(marker);
            }
            marker_pub_->publish(slam_markers_);
        }

        /// \brief Publish transforms
        void publish_transforms() {
            auto stamp = this->now();

            // T_map_odom = T_map_base * T_odom_base^{-1}
            // so that map->odom->base_footprint chains to the EKF estimate
            turtlelib::Transform2D T_map_base(
                turtlelib::Vector2D{ekf_.get_x(), ekf_.get_y()}, ekf_.get_theta());
            turtlelib::Transform2D T_odom_base(
                turtlelib::Vector2D{x_, y_}, theta_);
            turtlelib::Transform2D T_map_odom = T_map_base * T_odom_base.inv();
            auto trans = T_map_odom.translation();
            auto rot   = T_map_odom.rotation();

            geometry_msgs::msg::TransformStamped tf_map_odom;
            tf_map_odom.header.stamp = stamp;
            tf_map_odom.header.frame_id = "map";
            tf_map_odom.child_frame_id = "odom";
            tf_map_odom.transform.translation.x = trans.x;
            tf_map_odom.transform.translation.y = trans.y;
            tf_map_odom.transform.translation.z = 0.0;
            tf_map_odom.transform.rotation.x = 0.0;
            tf_map_odom.transform.rotation.y = 0.0;
            tf_map_odom.transform.rotation.z = std::sin(rot / 2.0);
            tf_map_odom.transform.rotation.w = std::cos(rot / 2.0);

            // map → blue/base_footprint: raw uncorrected odometry
            // shows where the robot thinks it is WITHOUT SLAM correction
            geometry_msgs::msg::TransformStamped tf_map_blue;
            tf_map_blue.header.stamp = stamp;
            tf_map_blue.header.frame_id = "nusim/world";
            tf_map_blue.child_frame_id = "blue/base_footprint";
            tf_map_blue.transform.translation.x = x_;
            tf_map_blue.transform.translation.y = y_;
            tf_map_blue.transform.translation.z = 0.0;
            tf_map_blue.transform.rotation.x = 0.0;
            tf_map_blue.transform.rotation.y = 0.0;
            tf_map_blue.transform.rotation.z = std::sin(theta_ / 2.0);
            tf_map_blue.transform.rotation.w = std::cos(theta_ / 2.0);

            tf_broadcaster_->sendTransform(tf_map_odom);
            tf_broadcaster_->sendTransform(tf_map_blue);
        }
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_marker_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        nav_msgs::msg::Path slam_path_;
        nav_msgs::msg::Path odom_path_;
        visualization_msgs::msg::MarkerArray slam_markers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SLAM>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
