#include "multinav_behaviors.h"

GooPose::GoToPose(const std::string &name,
                  const BT::NodeConfiguration &config,
                  rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
    {

    }

    BT::PortList GoToPose::providePorts()
    {

    }

    BT::NodeStatus GoToPose::onStart()
    {

    }

    BT::NodeStatus GoToPose::onRunning()
    {

    }

    void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrapperResult &result)
    {
        
    }
                  