#include "rclcpp/rclcpp.hpp"

class SLAM : public rclcpp::Node {
    public:
        SLAM() : Node("slam") {
            
        }
    private:
        
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SLAM>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
