#include "multinav.h"

MultiNav::MultiNav(const std::string &node_name) : Node(nodeName)
{
    RCLCPP_INFO(get_logger(), "Init Done")
}

void MultiNav::setup()
{

}

void MultiNav::create_behavior_tree()
{

}

void MultiNav::update_behavior_tree()
{

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiNav>("multi_nav");
    rclcpp::spin(node);
    rclcpp::shutdown();

}