#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;

class ElevatorLoading : public rclcpp::Node
{
  public:
    ElevatorLoading()
    : Node("elevator_loading")
    {
      laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&ElevatorLoading::laser_callback, this, _1));
    }

  private:
    void laser_callback(const sensor_msgs::msg::LaserScan & msg)
    {
      laser_scan = msg;
      //RCLCPP_INFO(this->get_logger(), "I heard");
      //RCLCPP_INFO(this->get_logger(), "first value: %f",laser_scan.ranges[0]);
    }

    sensor_msgs::msg::LaserScan laser_scan;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevatorLoading>());
  rclcpp::shutdown();
  return 0;
}