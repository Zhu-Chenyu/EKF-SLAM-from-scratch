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
  NuSimulator()
  : Node("nusimulator")
  {
    this->declare_parameter("rate", 100.0);
    double rate = this->get_parameter("rate").as_double();
    auto dt = std::chrono::duration<double>(1.0 / rate);
    ts_publisher = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    reset_srv = this->create_service<std_srvs::srv::Empty>(
        "~/reset", std::bind(&NuSimulator::reset_callback, this,
          std::placeholders::_1,
          std::placeholders::_2));
    timer_ = this->create_wall_timer(
      dt, std::bind(&NuSimulator::timer_callback, this));

      // Broadcast tf between "nusim/world" and "red/base_footprint"
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);
    x0_ = this->get_parameter("x0").as_double();
    y0_ = this->get_parameter("y0").as_double();
    theta0_ = this->get_parameter("theta0").as_double();
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Visualize Arena Boundary in RViz
    this->declare_parameter("arena_x_length", 8.0);
    this->declare_parameter("arena_y_length", 8.0);
    double arena_x_length = this->get_parameter("arena_x_length").as_double();
    double arena_y_length = this->get_parameter("arena_y_length").as_double();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    wall_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_walls",
      qos);

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers = generate_arena_markers(arena_x_length, arena_y_length);
    wall_marker_pub_->publish(marker_array);

      //Visualize Obstacles in RViz
    obs_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("~/real_obstacles", qos);
    this->declare_parameter("obstacles.x", std::vector<double>{});
    this->declare_parameter("obstacles.y", std::vector<double>{});
    this->declare_parameter("obstacles.r", 0.2);
    Obstacle obs;
    obs.x = this->get_parameter("obstacles.x").as_double_array();
    obs.y = this->get_parameter("obstacles.y").as_double_array();
    obs.radius = this->get_parameter("obstacles.r").as_double();
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
    }
  }

private:
  uint64_t timestep = 0;

    // Robot pose state
  double x_;
  double y_;
  double theta_;

    // Initial pose for reset
  double x0_;
  double y0_;
  double theta0_;

    // Parameters for obstacles
  struct Obstacle
  {
    std::vector<double> x;
    std::vector<double> y;
    double radius;
  };

  void timer_callback()
  {
    timestep++;
    auto message = std_msgs::msg::UInt64();
    message.data = timestep;
    RCLCPP_INFO(this->get_logger(), "Publishing timestep: '%lu'", message.data);
    ts_publisher->publish(message);
    broadcast_tf();
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

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr ts_publisher;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_marker_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSimulator>());
  rclcpp::shutdown();
  return 0;
}
