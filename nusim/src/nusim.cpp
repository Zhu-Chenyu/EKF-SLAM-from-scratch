#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

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
    }

  private:
    uint64_t timestep = 0;

    void timer_callback()
    {
      timestep ++;
      auto message = std_msgs::msg::UInt64();
      message.data = timestep;
      RCLCPP_INFO(this->get_logger(), "Publishing timestep: '%lu'", message.data);
      ts_publisher->publish(message);
    }

    void reset_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO(this->get_logger(), "Resetting timestep to 0");
      timestep = 0;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr ts_publisher;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSimulator>());
  rclcpp::shutdown();
  return 0;
}