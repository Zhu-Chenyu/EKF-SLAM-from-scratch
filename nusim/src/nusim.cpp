#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
    NuSimulator()
    : Node("nusimulator")
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
        auto arena_x_length = get_parameter("arena_x_length").as_double();
        auto arena_y_length = get_parameter("arena_y_length").as_double();
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        wall_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls",
          qos);

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers = generate_arena_markers(arena_x_length, arena_y_length);
        wall_marker_pub_->publish(marker_array);

          //Visualize Obstacles in RViz
        obs_marker_pub_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", qos);
        declare_parameter("obstacles.x", std::vector<double>{});
        declare_parameter("obstacles.y", std::vector<double>{});
        declare_parameter("obstacles.r", 0.2);
        
        obs.x = get_parameter("obstacles.x").as_double_array();
        obs.y = get_parameter("obstacles.y").as_double_array();
        obs.radius = get_parameter("obstacles.r").as_double();
        if (obs.x.size() != obs.y.size()) {
            RCLCPP_ERROR(this->get_logger(), "Obstacle x and y size mismatch!");
            throw std::runtime_error("Obstacle x and y size mismatch!");
        } else {
            visualization_msgs::msg::MarkerArray obstacle_markers;
            int id = 0;
            for (size_t i = 0; i < obs.x.size(); i++) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "nusim/world";
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "obstacles";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = obs.x.at(i);
                marker.pose.position.y = obs.y.at(i);
                marker.pose.position.z = 0.25 / 2.0;
                marker.scale.x = obs.radius * 2;
                marker.scale.y = obs.radius * 2;
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
        dd = turtlelib::DiffDrive(track_width, wheel_radius);

        cmd_sub_ = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
          "red/wheel_cmd", 10, std::bind(&NuSimulator::cmd_callback, this, std::placeholders::_1));
        sensor_pub_ = this->create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("red/joint_states", 10);

        //add noise to simulation
        declare_parameter("input_noise", 0.0);
        declare_parameter("slip_fraction", 0.0);
        get_parameter("input_noise", this->input_noise_);
        get_parameter("slip_fraction", this->slip_fraction_)  ;
        gen_(std::random_device{}());
        wheel_noise_distribution_ = std::normal_distribution<double>(0.0, std::sqrt(input_noise_));
        slip_distribution_ = std::uniform_real_distribution<double>(-slip_fraction_, slip_fraction_);

        // lidar sensor
        declare_parameter("basic_sensor_variance", 0.0);
        declare_parameter("max_range", 0.0);
        get_parameter("basic_sensor_variance", this->basic_sensor_variance_);
        get_parameter("max_range", this->max_range_);

        // sensor obstacles marker
        sensor_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_sensor", qos);
        sensor_noise_distribution_ = std::normal_distribution<double>(0.0, std::sqrt(basic_sensor_variance_));


    }

private:
    uint64_t timestep = 0;
    std::chrono::duration<double> dt_{0.01};
    auto dt_seconds_ = 0.01;
    auto motor_cmd_per_rad_sec_ = 0.024;
    auto encoder_ticks_per_rad_ = 0.0;

    // Robot pose state
    double x_;
    double y_;
    double theta_;

    // Initial pose for reset
    double x0_;
    double y0_;
    double theta0_;

    // Wheel state
    auto v_left_ = 0.0;
    auto v_right_ = 0.0;
    auto pos_left_ = 0.0;
    auto pos_right_ = 0.0;

    // Noises
    auto input_noise_ = 0.0;
    auto slip_fraction_ = 0.0;
    std::mt19937 gen_;
    std::normal_distribution<double> wheel_noise_distribution_;
    std::uniform_real_distribution<double> slip_distribution_;

    // Slip
    auto pos_left_slip_ = 0.0;
    auto pos_right_slip_ = 0.0;
    auto vel_left_slip_ = 0.0;
    auto vel_right_slip_ = 0.0;

    // Lidar sensor
    auto basic_sensor_variance_ = 0.0;
    auto max_range_ = 0.0;
    std::normal_distribution<double> sensor_noise_distribution_;

    turtlelib::DiffDrive dd;

    // Parameters for obstacles(array of obstacles)
    struct Obstacle
    {
        std::vector<double> x;
        std::vector<double> y;
        double radius;
    };
    Obstacle obs;

    void timer_callback()
    {
        timestep++;
        auto message = std_msgs::msg::UInt64();
        message.data = timestep;
        // RCLCPP_INFO(this->get_logger(), "Publishing timestep: '%lu'", message.data);
        ts_publisher->publish(message);
        broadcast_tf();

        pos_left_ += v_left_ * dt_seconds_;
        pos_right_ += v_right_ * dt_seconds_;
        pos_left_slip_ += vel_left_slip_ * dt_seconds_;
        pos_right_slip_ += vel_right_slip_ * dt_seconds_;

        this->dd.forward_kinematics(pos_left_, pos_right_);
        x_ = this->dd.get_x();
        y_ = this->dd.get_y();
        theta_ = this->dd.get_theta();

        nuturtlebot_msgs::msg::SensorData sensor_msg;
        sensor_msg.left_encoder = pos_left_slip_ * encoder_ticks_per_rad_;
        sensor_msg.right_encoder = pos_right_slip_ * encoder_ticks_per_rad_;
        sensor_pub_->publish(sensor_msg);

        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.name = {"wheel_left_joint", "wheel_right_joint"};
        joint_msg.position = {pos_left_, pos_right_};
        joint_msg.velocity = {v_left_, v_right_};
        joint_state_pub_->publish(joint_msg);

        if (timestep % 20 == 0) {
            visualization_msgs::msg::MarkerArray marker_array;
            for (size_t i = 0; i < obs.x.size(); i++) {
                auto distance = std::sqrt(std::pow(obs.x[i] - x_, 2) + std::pow(obs.y[i] - y_, 2));
                visualization_msgs::msg::Marker marker;
                marker.header.stamp = this->get_clock()->now();
                marker.header.frame_id = "red/base_footprint";
                marker.ns = "obstacles";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                // add marker if distance is less than max_range_ and add noise to the marker
                if (distance < max_range_) {
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    auto dx = obs.x[i] - x_;
                    auto dy = obs.y[i] - y_;
                    auto rel_x = std::cos(theta_) * dx + std::sin(theta_) * dy;
                    auto rel_y = -std::sin(theta_) * dx + std::cos(theta_) * dy;
                    marker.pose.position.x = rel_x + sensor_noise_distribution_(gen_);
                    marker.pose.position.y = rel_y + sensor_noise_distribution_(gen_);
                    marker.pose.position.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = obs.radius * 2;
                    marker.scale.y = obs.radius * 2;
                    marker.scale.z = 0.1;
                    marker.color.a = 1.0;
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
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
      const double wall_thickness = 0.1;
      const double wall_height = 0.25;
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

        // The velocity commands are given in the range [-265, 265], which corresponds to [-2.84, 2.84] in the raw command values.
        v_left_ = v_left_raw / 265.0 * 2.84;
        v_right_ = v_right_raw / 265.0 * 2.84;

        if (v_left_ != 0){
          v_left_ += wheel_noise_distribution_(gen_);
        }
        if (v_right_ != 0){
          v_right_ += wheel_noise_distribution_(gen_);
        }
        vel_left_slip_ = v_left_ * (1 + slip_distribution_(gen_));
        vel_right_slip_ = v_right_ * (1 + slip_distribution_(gen_));

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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuSimulator>());
    rclcpp::shutdown();
    return 0;
}
