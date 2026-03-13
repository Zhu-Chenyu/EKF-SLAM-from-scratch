#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Landmark : public rclcpp::Node {
public:
    Landmark() : rclcpp::Node("landmark") {
        declare_parameter("cluster_threshold", 0.1);
        cluster_threshold_ = get_parameter("cluster_threshold").as_double();
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "red/scan", 10, std::bind(&Landmark::scan_callback, this, std::placeholders::_1));
    }
private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        clusters_.x.clear();
        clusters_.y.clear();
        auto angle_increment = msg->angle_increment;
        auto angle_start = msg->angle_min;

        //process the first scan point
        if (std::isinf(msg->ranges.at(0))) {
            msg->ranges.at(0) = 1e9;
        }
        auto x = msg->ranges.at(0) * std::cos(angle_start);
        auto y = msg->ranges.at(0) * std::sin(angle_start);
        clusters_.x.push_back(std::vector<double>{x});
        clusters_.y.push_back(std::vector<double>{y});
        auto prev_id = 0;
        auto prev_x = x;
        auto prev_y = y;

        for (int i = 1; i < msg->ranges.size(); i++) {
            if (std::isinf(msg->ranges.at(i))) {
                msg->ranges.at(i) = 1e9;
            }
            auto x = msg->ranges.at(i) * std::cos(angle_start + i * angle_increment);
            auto y = msg->ranges.at(i) * std::sin(angle_start + i * angle_increment);
            if (std::sqrt(std::pow(x - prev_x, 2) + std::pow(y - prev_y, 2)) < cluster_threshold_) {
                clusters_.x.at(prev_id).push_back(x);
                clusters_.y.at(prev_id).push_back(y);
            } else {
                clusters_.x.push_back(std::vector<double>{x});
                clusters_.y.push_back(std::vector<double>{y});
                prev_id++;
            }
            prev_x = x;
            prev_y = y;
        }

        //check if the last cluster is connected to the first cluster
        if (std::sqrt(std::pow(clusters_.x.at(0).at(0) - clusters_.x.back().back(), 2) + std::pow(clusters_.y.at(0).at(0) - clusters_.y.back().back(), 2)) < cluster_threshold_) {
            clusters_.x.at(0).insert(clusters_.x.at(0).end(), clusters_.x.back().begin(), clusters_.x.back().end());
            clusters_.y.at(0).insert(clusters_.y.at(0).end(), clusters_.y.back().begin(), clusters_.y.back().end());
            clusters_.x.pop_back();
            clusters_.y.pop_back();
        }

        for (auto i=0; i<clusters_.x.size(); i++) {
            if (clusters_.x.at(i).size() < 3) {
                clusters_.x.erase(clusters_.x.begin() + i);
                clusters_.y.erase(clusters_.y.begin() + i);
                i--;
            }
        }
    }

    struct point_clusters {
        std::vector<std::vector<double>> x;
        std::vector<std::vector<double>> y;
    };
    point_clusters clusters_;
    
    struct Obstacle
    {
        std::vector<double> x;
        std::vector<double> y;
        double radius;
    };
    Obstacle obs_;

    double cluster_threshold_ = 0.1;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Landmark>());
    rclcpp::shutdown();
    return 0;
}
