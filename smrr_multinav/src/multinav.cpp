#include "multinav.h"

using namespace std::chrono_literals;

const std::string bt_xml_dir = 
        ament_index_cpp::get_package_share_directory("smrr_multinav") + "/bt_xml";

MultiNav::MultiNav(const std::string &node_name) : Node(node_name)
{   
    this->declare_parameter("multinav_config", "none");
    this->declare_parameter("elevator_config", "none");
    this->declare_parameter("start_from_dock", false);
    RCLCPP_INFO(get_logger(), "Init Done");
}

void MultiNav::setup()
{
    create_behavior_tree();

    const auto timer_period = 500ms;
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&MultiNav::update_behavior_tree, this)
    );

}

void MultiNav::create_behavior_tree()
{
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder go_to_pose_builder =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    };
    factory.registerBuilder<GoToPose>("GoToPose", go_to_pose_builder);

    BT::NodeBuilder load_map_builder =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<LoadMapFromSlam>(name, config, shared_from_this());
    };
    factory.registerBuilder<LoadMapFromSlam>("LoadMapFromSlam", load_map_builder);

    BT::NodeBuilder wait_event_builder =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<WaitEvent>(name, config, shared_from_this());
    };
    factory.registerBuilder<WaitEvent>("WaitEvent", wait_event_builder);

    BT::NodeBuilder load_params_builder =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<LoadParams>(name, config, shared_from_this());
    };
    factory.registerBuilder<LoadParams>("LoadParams", load_params_builder);

    BT::NodeBuilder elevator_loading_builder =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<ElevatorLoading>(name, config, shared_from_this());
    };
    factory.registerBuilder<ElevatorLoading>("ElevatorLoading", elevator_loading_builder);

    BT::NodeBuilder multi_floor_goal_builder =
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<MultiFloorGoal>(name, config, shared_from_this());
    };
    factory.registerBuilder<MultiFloorGoal>("MultiFloorGoal", multi_floor_goal_builder);
    
    tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");
}

void MultiNav::update_behavior_tree()
{
    BT::NodeStatus tree_status = tree_.tickRoot();

    if (tree_status == BT::NodeStatus::RUNNING)
    {
        return;
    }
    else if (tree_status == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "BT completed");
    }
    else if (tree_status == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO(this->get_logger(), "BT failed");
        timer_->cancel();
    }
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiNav>("multi_nav");
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();

}