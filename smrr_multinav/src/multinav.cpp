#include "multinav.h"

MultiNav::MultiNav(const std::string &node_name) : Node(nodeName)
{
    RCLCPP_INFO(get_logger(), "Init Done")
}

void MultiNav::setup()
{
    create_behavior_tree();

    const auto timer_period = 500ms;
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&MultiNav::update_behavior_tree, this);
    )

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
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();

}