#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ElevatorLoading : public rclcpp::Node
{
public:
    ElevatorLoading()
    : Node("elevator_loading"), A(Eigen::MatrixXf::Ones(20, 2)), B(Eigen::VectorXf::Zero(20))
    {
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ElevatorLoading::laser_callback, this, _1));
        timer_ = this->create_wall_timer(500ms, std::bind(&ElevatorLoading::set_orientation, this));
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan & msg)
    {
        laser_scan = msg;
    }

    void set_orientation()
    {
        if (laser_scan.ranges.size() < 250) {
            RCLCPP_WARN(this->get_logger(), "Not enough laser scan data received.");
            return;
        }

        std::vector<float> laser_slice(laser_scan.ranges.begin() + 149, laser_scan.ranges.begin() + 250);
        if (laser_slice.size() < 100) { 
            RCLCPP_WARN(this->get_logger(), "Laser slice does not contain enough elements.");
            return;
        }

        filtered_points.clear();
        for (int i = 0; i < 20; i++) 
        {
            if (5 * i + 4 < laser_slice.size()) {
                std::sort(laser_slice.begin() + 5 * i, laser_slice.begin() + 5 * i + 5); // Sort dynamic window of 5 elements
                filtered_points.push_back(laser_slice[5 * i + 2]); // Get the middle reading as median
                A(i, 1) = laser_slice[5 * i + 2] * cos((-28.8 + 3 * i) * (M_PI / 180.0));
                B(i) = laser_slice[5 * i + 2] * sin((-28.8 + 3 * i) * (M_PI / 180.0));
            }
        }

        Eigen::VectorXf X = ((A.transpose()*A).inverse())*A.transpose()*B;
        std::cout<<X[0] <<" "<<X[1]<<std::endl;
    }

    Eigen::MatrixXf A; 
    Eigen::VectorXf B; 
    Eigen::VectorXf X; 
    std::vector<float> filtered_points;
    sensor_msgs::msg::LaserScan laser_scan;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElevatorLoading>());
    rclcpp::shutdown();
    return 0;
}