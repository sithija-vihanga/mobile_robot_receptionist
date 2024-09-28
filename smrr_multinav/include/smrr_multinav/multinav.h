#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory"

class MultiNav : public rclcpp::Node
{
    public:
        explicit MultiNav(const std::string &node_name);
        void setup();
        void create_behavior_tree();
        void update_behavior_tree();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        BT::Tree tree_;

};