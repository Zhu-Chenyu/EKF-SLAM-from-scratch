/// \file
/// \brief A ROS2 node that simulates the turtlebot3 robot
///
/// PARAMETERS:
///     rate (double): the rate at which the simulation runs (Hz)
///     x0 (double): initial x position of the robot
///     y0 (double): initial y position of the robot
///     theta0 (double): initial orientation of the robot
///     arena_x_length (double): length of the arena in the x direction
///     arena_y_length (double): length of the arena in the y direction
///     obstacles.x (double[]): x coordinates of obstacles
///     obstacles.y (double[]): y coordinates of obstacles
///     obstacles.r (double): radius of the obstacles
///     input_noise (double): variance of the noise added to wheel velocities
///     slip_fraction (double): fraction of wheel slip
///     basic_sensor_variance (double): variance of the basic obstacle sensor
///     max_range (double): maximum detection range for obstacles
///     collision_radius (double): collision radius of the robot
///     draw_only (bool): if true, only draw the arena and obstacles
///     encoder_ticks_per_rad (double): encoder ticks per radian
///     wheel_radius (double): radius of the wheels
///     track_width (double): distance between the wheels
///     motor_cmd_per_rad_sec (double): motor command per rad/sec
///     scan_noise (double): noise added to the lidar scan
///     scan_angle_increment (double): angle increment of the lidar
///     scan_resolution (double): resolution of the lidar
///     scan_range_min (double): minimum range of the lidar
///     scan_range_max (double): maximum range of the lidar
/// PUBLISHES:
///     ~/timestep (std_msgs/msg/UInt64): the current simulation timestep
///     ~/real_walls (visualization_msgs/msg/MarkerArray): arena wall markers
///     ~/real_obstacles (visualization_msgs/msg/MarkerArray): ground-truth obstacle markers
///     ~/fake_sensor (visualization_msgs/msg/MarkerArray): simulated sensor obstacle detections
///     red/sensor_data (nuturtlebot_msgs/msg/SensorData): simulated encoder data
///     red/joint_states (sensor_msgs/msg/JointState): simulated joint states
///     red/scan (sensor_msgs/msg/LaserScan): simulated lidar scan
///     ground_truth (nav_msgs/msg/Path): the ground-truth path of the robot
/// SUBSCRIBES:
///     red/wheel_cmd (nuturtlebot_msgs/msg/WheelCommands): wheel velocity commands
/// SERVERS:
///     ~/reset (std_srvs/srv/Empty): resets the simulation timestep and robot pose

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <random>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/angle.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class NuSimulator : public rclcpp::Node
{
public:
    /// \brief Create a NuSimulator node
    /// \param rate The rate at which the timestep is published
    /// \param x0 Initial x position of the robot
    /// \param y0 Initial y position of the robot
    /// \param theta0 Initial orientation of the robot
    /// \param arena_x_length Length of the arena in the x direction
    /// \param arena_y_length Length of the arena in the y direction
    /// \param obs_x Vector of x positions of obstacles
    /// \param obs_y Vector of y positions of obstacles
    /// \param obs_radius Radius of the obstacles(all obstacles have the same radius)
    /// \param basic_sensor_variance Variance of the basic sensor
    /// \param max_range Maximum range of the basic sensor
    /// \param scan_noise Noise added to the scan
    /// \param draw_only Only draw the arena and obstacles
    NuSimulator()
    : Node("nusimulator"),
    gen_(std::random_device{}())
    {
        declare_parameter("rate", 100.0);
        auto rate = get_parameter("rate").as_double();
        dt_ = std::chrono::duration<double>(1.0 / rate);
        ts_publisher = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        reset_srv = this->create_service<std_srvs::srv::Empty>(
            "~/reset", std::bind(&NuSimulator::reset_callback, this,
              std::placeholders::_1,
              std::placeholders::_2));
        timer_ = this->create_wall_timer(
          dt_, std::bind(&NuSimulator::timer_callback, this));

        // Broadcast tf between "nusim/world" and "red/base_footprint"
        declare_parameter("x0", 0.0);
        declare_parameter("y0", 0.0);
        declare_parameter("theta0", 0.0);
        x0_ = get_parameter("x0").as_double();
        y0_ = get_parameter("y0").as_double();
        theta0_ = get_parameter("theta0").as_double();
        x_ = x0_;
        y_ = y0_;
        theta_ = theta0_;
        tf_broadcaster_ =
          std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Visualize Arena Boundary in RViz
        declare_parameter("arena_x_length", 8.0);
        declare_parameter("arena_y_length", 8.0);
        get_parameter("arena_x_length", this->arena_x_length_);
        get_parameter("arena_y_length", this->arena_y_length_);
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        wall_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls",
          qos);

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers = generate_arena_markers(this->arena_x_length_, this->arena_y_length_);
        wall_marker_pub_->publish(marker_array);

        //Visualize Obstacles in RViz
        obs_marker_pub_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", qos);
        declare_parameter("obstacles.x", std::vector<double>{});
        declare_parameter("obstacles.y", std::vector<double>{});
        declare_parameter("obstacles.r", 0.2);
        
        get_parameter("obstacles.x", this->obs_.x);
        get_parameter("obstacles.y", this->obs_.y);
        get_parameter("obstacles.r", this->obs_.radius);
        if (obs_.x.size() != obs_.y.size()) {
            RCLCPP_ERROR(this->get_logger(), "Obstacle x and y size mismatch!");
            throw std::runtime_error("Obstacle x and y size mismatch!");
        } else {
            visualization_msgs::msg::MarkerArray obstacle_markers;
            int id = 0;
            for (size_t i = 0; i < obs_.x.size(); i++) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "nusim/world";
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "obstacles";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = obs_.x.at(i);
                marker.pose.position.y = obs_.y.at(i);
                marker.pose.position.z = 0.25 / 2.0;
                marker.scale.x = obs_.radius * 2;
                marker.scale.y = obs_.radius * 2;
                marker.scale.z = 0.25;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;

                obstacle_markers.markers.push_back(marker);
            }
            obs_marker_pub_->publish(obstacle_markers);

            declare_parameter("encoder_ticks_per_rad", 0.0);
            get_parameter("encoder_ticks_per_rad", this->encoder_ticks_per_rad_);

        }

        declare_parameter("wheel_radius", 0.033);
        declare_parameter("track_width", 0.16);
        declare_parameter("motor_cmd_per_rad_sec", 0.024);
        auto wheel_radius = get_parameter("wheel_radius").as_double();
        auto track_width = get_parameter("track_width").as_double();
        motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
        dt_seconds_ = 1.0 / rate;
        dd_ = turtlelib::DiffDrive(track_width, wheel_radius);

        cmd_sub_ = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
          "red/wheel_cmd", 10, std::bind(&NuSimulator::cmd_callback, this, std::placeholders::_1));
        sensor_pub_ = this->create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);

        declare_parameter("draw_only", false);
        get_parameter("draw_only", this->draw_only_);

        //add noise to simulation
        declare_parameter("input_noise", 0.01);
        declare_parameter("slip_fraction", 0.01);
        get_parameter("input_noise", this->input_noise_);
        get_parameter("slip_fraction", this->slip_fraction_)  ;
        
        wheel_noise_distribution_ = std::normal_distribution<double>(0.0, std::sqrt(input_noise_));
        slip_distribution_ = std::uniform_real_distribution<double>(-slip_fraction_, slip_fraction_);

        // lidar sensor
        declare_parameter("basic_sensor_variance", 0.01);
        declare_parameter("max_range", 2.0);
        get_parameter("basic_sensor_variance", this->basic_sensor_variance_);
        get_parameter("max_range", this->max_range_);

        // sensor obstacles marker
        sensor_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_sensor", 10);
        sensor_noise_distribution_ = std::normal_distribution<double>(0.0, std::sqrt(basic_sensor_variance_));

        // obstacles collision
        declare_parameter("collision_radius", 0.11);
        get_parameter("collision_radius", this->collision_radius_);

        // ground truth path
        ground_truth_pub_ = this->create_publisher<nav_msgs::msg::Path>("ground_truth", 10);
        ground_truth_path_.header.frame_id = "nusim/world";

        // laser scan
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("red/scan", 10);
        declare_parameter("scan_noise", 0.001);
        declare_parameter("scan_angle_increment", turtlelib::deg2rad(1.0));  // 1 degree
        declare_parameter("scan_resolution", turtlelib::deg2rad(1.0));  // 1 degree
        declare_parameter("scan_range_min", 0.12);  // 120mm
        declare_parameter("scan_range_max", 3.5);  // 3500mm
        get_parameter("scan_noise", this->scan_noise_);
        get_parameter("scan_angle_increment", this->scan_angle_increment_);
        get_parameter("scan_resolution", this->scan_resolution_);
        get_parameter("scan_range_min", this->scan_range_min_);
        get_parameter("scan_range_max", this->scan_range_max_);
        scan_noise_distribution_ = std::normal_distribution<double>(0.0, std::sqrt(scan_noise_));



    }

private:
    uint64_t timestep = 0;
    std::chrono::duration<double> dt_{0.01};
    double dt_seconds_ = 0.01;
    double motor_cmd_per_rad_sec_ = 0.024;
    double encoder_ticks_per_rad_ = 0.0;

    // Robot pose state
    double x_;
    double y_;
    double theta_;

    // Initial pose for reset
    double x0_;
    double y0_;
    double theta0_;

    // Wheel state
    double v_left_ = 0.0;
    double v_right_ = 0.0;
    double pos_left_ = 0.0;
    double pos_right_ = 0.0;

    // Arena
    const double wall_thickness = 0.1;
    const double wall_height = 0.25;
    double arena_x_length_ = 0.0;
    double arena_y_length_ = 0.0;

    // Noises
    double input_noise_ = 0.0;
    double slip_fraction_ = 0.0;
    std::mt19937 gen_;
    std::normal_distribution<double> wheel_noise_distribution_;
    std::uniform_real_distribution<double> slip_distribution_;

    // Slip
    double pos_left_slip_ = 0.0;
    double pos_right_slip_ = 0.0;
    double vel_left_slip_ = 0.0;
    double vel_right_slip_ = 0.0;

    // Lidar sensor
    double basic_sensor_variance_ = 0.0;
    double max_range_ = 0.0;
    std::normal_distribution<double> sensor_noise_distribution_;

    // Collision radius
    double collision_radius_ = 0.0;

    // Laser scan
    double scan_noise_ = 0.0;
    double scan_angle_increment_ = 0.0;
    double scan_resolution_ = 0.0;
    double scan_range_min_ = 0.0;
    double scan_range_max_ = 0.0;
    std::normal_distribution<double> scan_noise_distribution_;

    bool draw_only_ = false;

    turtlelib::DiffDrive dd_;

    // Parameters for obstacles(array of obstacles)
    struct Obstacle
    {
        std::vector<double> x;
        std::vector<double> y;
        double radius;
    };
    Obstacle obs_;

    void timer_callback()
    {
        timestep++;
        auto message = std_msgs::msg::UInt64();
        message.data = timestep;
        ts_publisher->publish(message);

        if (draw_only_) {
            return;
        }

        broadcast_tf();

        // ground truth path
        geometry_msgs::msg::PoseStamped gt_pose;
        gt_pose.header.stamp = this->get_clock()->now();
        gt_pose.header.frame_id = "nusim/world";
        gt_pose.pose.position.x = x_;
        gt_pose.pose.position.y = y_;
        tf2::Quaternion q_gt;
        q_gt.setRPY(0, 0, theta_);
        gt_pose.pose.orientation.x = q_gt.x();
        gt_pose.pose.orientation.y = q_gt.y();
        gt_pose.pose.orientation.z = q_gt.z();
        gt_pose.pose.orientation.w = q_gt.w();
        ground_truth_path_.header.stamp = this->get_clock()->now();
        ground_truth_path_.poses.push_back(gt_pose);
        ground_truth_pub_->publish(ground_truth_path_);

        pos_left_ += v_left_ * dt_seconds_;
        pos_right_ += v_right_ * dt_seconds_;
        pos_left_slip_ += vel_left_slip_ * dt_seconds_;
        pos_right_slip_ += vel_right_slip_ * dt_seconds_;

        dd_.forward_kinematics(pos_left_, pos_right_);
        x_ = dd_.get_x();
        y_ = dd_.get_y();
        theta_ = dd_.get_theta();

        // check collision
        for (size_t i = 0; i < obs_.x.size(); i++) {
            auto dist_to_obs = std::hypot(obs_.x.at(i) - x_, obs_.y.at(i) - y_);
            if (dist_to_obs < collision_radius_ + obs_.radius) {
                x_ = obs_.x.at(i) + (collision_radius_ + obs_.radius)/dist_to_obs * (x_ - obs_.x.at(i));
                y_ = obs_.y.at(i) + (collision_radius_ + obs_.radius)/dist_to_obs * (y_ - obs_.y.at(i));
                dd_.set_x(x_);
                dd_.set_y(y_);
                dd_.set_theta(theta_);
            }
        }

        nuturtlebot_msgs::msg::SensorData sensor_msg;
        sensor_msg.left_encoder = pos_left_ * encoder_ticks_per_rad_;
        sensor_msg.right_encoder = pos_right_ * encoder_ticks_per_rad_;
        sensor_pub_->publish(sensor_msg);

        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.name = {"wheel_left_joint", "wheel_right_joint"};
        joint_msg.position = {pos_left_slip_, pos_right_slip_};
        joint_msg.velocity = {v_left_, v_right_};
        joint_state_pub_->publish(joint_msg);

        if (timestep % 20 == 0) {
            // publish sensor obstacles marker
            visualization_msgs::msg::MarkerArray marker_array;
            for (size_t i = 0; i < obs_.x.size(); i++) {
                auto distance = std::hypot(obs_.x.at(i) - x_, obs_.y.at(i) - y_);
                visualization_msgs::msg::Marker marker;
                marker.header.stamp = rclcpp::Time(0);
                marker.header.frame_id = "red/base_footprint";
                marker.ns = "obstacles";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                // add marker if distance is less than max_range_ and add noise to the marker
                if (distance < max_range_) {
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    auto dx = obs_.x.at(i) - x_;
                    auto dy = obs_.y.at(i) - y_;
                    auto rel_x = std::cos(theta_) * dx + std::sin(theta_) * dy;
                    auto rel_y = -std::sin(theta_) * dx + std::cos(theta_) * dy;
                    marker.pose.position.x = rel_x + sensor_noise_distribution_(gen_);
                    marker.pose.position.y = rel_y + sensor_noise_distribution_(gen_);
                    marker.pose.position.z = 0.25 / 2.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = obs_.radius * 2;
                    marker.scale.y = obs_.radius * 2;
                    marker.scale.z = 0.25;
                    marker.color.a = 1.0;
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker_array.markers.push_back(marker);
                }
                // delete marker if distance is greater than max_range
                else{
                    marker.action = visualization_msgs::msg::Marker::DELETE;
                    marker_array.markers.push_back(marker);
                }
            }
            sensor_marker_pub_->publish(marker_array);

            // publish laser scan
            sensor_msgs::msg::LaserScan scan;
            scan.header.stamp = this->get_clock()->now();
            scan.header.frame_id = "red/base_footprint";
            scan.angle_min = -M_PI;
            scan.angle_max = M_PI;
            scan.angle_increment = scan_angle_increment_;
            scan.time_increment = 0.0;
            scan.scan_time = 20.0 * dt_seconds_;
            scan.range_min = scan_range_min_;
            scan.range_max = scan_range_max_;
            scan.ranges.resize((scan.angle_max - scan.angle_min) / scan.angle_increment + 1);
            for (size_t i = 0; i < (scan.angle_max - scan.angle_min) / scan.angle_increment + 1; i++) {
                auto laser_angle = scan.angle_min + i * scan.angle_increment + theta_;
                auto result = std::numeric_limits<double>::infinity();
                // for each obstacle, check if the laser beam intersects with the obstacle
                for (size_t j = 0; j < obs_.x.size(); j++) {
                    auto dx = obs_.x.at(j) - x_;
                    auto dy = obs_.y.at(j) - y_;
                    auto dist_to_obs = std::hypot(dx, dy);
                    if (dist_to_obs < scan_range_max_) {
                        auto angle_to_obs = std::atan2(dy, dx);
                        auto laser_obs_angle = std::abs(angle_to_obs - laser_angle);
                        if (laser_obs_angle > M_PI / 2) {
                            continue;
                        }
                        if (dist_to_obs * std::sin(laser_obs_angle) < obs_.radius) {
                            result = std::min(result, dist_to_obs * std::cos(laser_obs_angle) - std::sqrt(obs_.radius * obs_.radius - dist_to_obs * dist_to_obs * std::sin(laser_obs_angle) * std::sin(laser_obs_angle)));
                        }
                    }
                }
                // for each wall, check if the laser beam intersects with the wall
                auto dy_up = arena_y_length_/2 - wall_thickness/2 - y_;
                auto dy_down = -arena_y_length_/2 + wall_thickness/2 - y_;
                auto dx_right = arena_x_length_/2 - wall_thickness/2 - x_;
                auto dx_left = -arena_x_length_/2 + wall_thickness/2 - x_;
                // check up wall
                if (std::sin(laser_angle) > 0) {
                    auto dist_up_wall = dy_up / std::sin(laser_angle);
                    result = std::min(result, dist_up_wall);
                }
                // check down wall
                if (std::sin(laser_angle) < 0) {
                    auto dist_down_wall = dy_down / std::sin(laser_angle);
                    result = std::min(result, dist_down_wall);
                }
                // check right wall
                if (std::cos(laser_angle) > 0) {
                    auto dist_right_wall = dx_right / std::cos(laser_angle);
                    result = std::min(result, dist_right_wall);
                }
                // check left wall
                if (std::cos(laser_angle) < 0) {
                    auto dist_left_wall = dx_left / std::cos(laser_angle);
                    result = std::min(result, dist_left_wall);
                }

                if (result > scan_range_max_ || result < scan_range_min_) {
                    result = std::numeric_limits<double>::infinity();  // out of range
                }
                else {
                    result += scan_noise_distribution_(gen_);  // add noise
                }
                scan.ranges.at(i) = result;
            }
            scan_pub_->publish(scan);
        }
    }

    void reset_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(this->get_logger(), "Resetting timestep to 0");
        timestep = 0;

        x0_ = this->get_parameter("x0").as_double();
        y0_ = this->get_parameter("y0").as_double();
        theta0_ = this->get_parameter("theta0").as_double();
        x_ = x0_;
        y_ = y0_;
        theta_ = theta0_;
    }

      // Broadcast tf between "nusim/world" and "red/base_footprint"
    void broadcast_tf()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "nusim/world";
        t.child_frame_id = "red/base_footprint";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

      // Generate arena wall markers
    std::vector<visualization_msgs::msg::Marker> generate_arena_markers(
        double x_length,
        double y_length)
    {

      std::vector<visualization_msgs::msg::Marker> markers;
        // Four walls
      std::vector<std::tuple<double, double, double, double>> walls = {
            // Top wall (positive y)
          {0.0, y_length / 2 + wall_thickness / 2, x_length + 2 * wall_thickness, wall_thickness},
            // Bottom wall (negative y)
          {0.0, -y_length / 2 - wall_thickness / 2, x_length + 2 * wall_thickness, wall_thickness},
            // Right wall (positive x)
          {x_length / 2 + wall_thickness / 2, 0.0, wall_thickness, y_length},
            // Left wall (negative x)
          {-x_length / 2 - wall_thickness / 2, 0.0, wall_thickness, y_length}
      };

      int id = 0;
      for (const auto & [x, y, x_scale, y_scale] : walls) {
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = "nusim/world";
          marker.header.stamp = this->get_clock()->now();
          marker.ns = "arena_walls";
          marker.id = id++;
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.pose.position.x = x;
          marker.pose.position.y = y;
          marker.pose.position.z = wall_height / 2;
          marker.scale.x = x_scale;
          marker.scale.y = y_scale;
          marker.scale.z = wall_height;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;

          markers.push_back(marker);
      }
      return markers;
    }

    void cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
    {
        double v_left_raw = msg->left_velocity;
        double v_right_raw = msg->right_velocity;

        // Convert motor commands to wheel angular velocities using motor_cmd_per_rad_sec parameter
        v_left_ = v_left_raw * motor_cmd_per_rad_sec_;
        v_right_ = v_right_raw * motor_cmd_per_rad_sec_;

        if (v_left_ != 0){
          v_left_ += wheel_noise_distribution_(gen_);
        }
        if (v_right_ != 0){
          v_right_ += wheel_noise_distribution_(gen_);
        }
        vel_left_slip_ = v_left_ * slip_distribution_(gen_);
        vel_right_slip_ = v_right_ * slip_distribution_(gen_);

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr ts_publisher;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_marker_pub_;
    rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ground_truth_pub_;
    nav_msgs::msg::Path ground_truth_path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuSimulator>());
    rclcpp::shutdown();
    return 0;
}
