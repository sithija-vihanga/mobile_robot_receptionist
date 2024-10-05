#include "multinav_behaviors.h"

GoToPose::GoToPose(const std::string &name,
                  const BT::NodeConfiguration &config,
                  rclcpp::Node::SharedPtr node_ptr)
        : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
    {
        action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
        done_flag_ = false;
    }

    BT::PortsList GoToPose::providedPorts()
    {
        return {BT::InputPort<std::string>("loc")};
    }

    BT::NodeStatus GoToPose::onStart()
    {
        BT::Optional<std::string> loc = getInput<std::string>("loc");
        const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

        YAML::Node locations = YAML::LoadFile(location_file);
        std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = pose[0];
        goal_msg.pose.pose.position.y = pose[1];

        tf2::Quaternion q;
        q.setRPY(0, 0, pose[2]);
        q.normalize();
        goal_msg.pose.pose.orientation = tf2::toMsg(q);

        done_flag_ = false;
        action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
        return BT::NodeStatus::RUNNING;


    }

    BT::NodeStatus GoToPose::onRunning()
    {
        if(done_flag_)
        {
            RCLCPP_INFO(node_ptr_->get_logger(), "[%s] goal reached\n",this->name());
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
    {
        if(result.result)
        {
            done_flag_ = true;
        }
    }


LoadMapFromSlam::LoadMapFromSlam(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    client_ = node_ptr_->create_client<slam_toolbox::srv::DeserializePoseGraph>("/slam_toolbox/deserialize_map");
}

BT::PortsList LoadMapFromSlam::providedPorts()
{
    return {BT::InputPort<std::string>("posegraph_file")};
}

BT::NodeStatus LoadMapFromSlam::onStart()
{
    BT::Optional<std::string> posegraph_file = getInput<std::string>("posegraph_file");
    const std::string location_file = node_ptr_->get_parameter("location_file").as_string();
    YAML::Node locations = YAML::LoadFile(location_file);
    std::string map_path = locations[posegraph_file.value()].as<std::string>();

    auto request = std::make_shared<slam_toolbox::srv::DeserializePoseGraph::Request>();
    RCLCPP_INFO(node_ptr_->get_logger(), " map file path: ", map_path);
    request->filename = "/home/sithija/mobile_receptionist_ws/src/smrr_localization/maps/floor05";
    request->match_type= 2;
    //request->initial_pose=geometry_msgs.msg.Pose2D(x=0.0, y=0.0, theta=0.0);


    future_ = client_->async_send_request(request);
    map_loading_done_flag_ = true; 
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LoadMapFromSlam::onRunning()
{
    if (map_loading_done_flag_)
    {
        return BT::NodeStatus::SUCCESS;
    }

    // if (future_.valid())
    // {
    //     auto status = future_.wait_for(std::chrono::milliseconds(10));
    //     if (status == std::future_status::ready)
    //     {
    //         auto result = future_.get();
    //         if (result->success)
    //         {
    //             map_loading_done_flag_ = true;
    //             return BT::NodeStatus::SUCCESS;
    //         }
    //         else
    //         {
    //             return BT::NodeStatus::FAILURE;
    //         }
    //     }
    // }
    return BT::NodeStatus::RUNNING;
}

void LoadMapFromSlam::onHalted()
{
    RCLCPP_INFO(node_ptr_->get_logger(), "Map loading was halted.");
}
                  