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

using namespace std::chrono_literals;

class NuSimulator : public rclcpp::Node
{
  public:
    /// \brief Create a NuSimulator node
    /// \param rate The rate at which the timestep is published
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

    void timer_callback()
    {
      timestep ++;
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

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr ts_publisher;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSimulator>());
  rclcpp::shutdown();
  return 0;
}