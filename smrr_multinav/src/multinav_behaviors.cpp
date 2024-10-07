#include "multinav_behaviors.h"

///////////////////////////////////////////////// Go to Pose //////////////////////////////////////////////////////////

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
///////////////////////////////////////////////// Load Map //////////////////////////////////////////////////////////

LoadMapFromSlam::LoadMapFromSlam(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr), tf_buffer_(node_ptr->get_clock()), tf_listener_(tf_buffer_)
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
        RCLCPP_INFO(node_ptr_->get_logger(), "Map file path: %s", map_path.c_str());
        request->filename = map_path; //"/home/sithija/mobile_receptionist_ws/src/smrr_localization/maps/floor05";
        request->match_type= 2;

        last_pose_ = this->getMapToBaseLink();
        RCLCPP_INFO(node_ptr_->get_logger(), "Transform from map to base_link: Translation: [%f, %f, %f], Rotation: [%f, %f, %f, %f]",
                last_pose_.transform.translation.x,
                last_pose_.transform.translation.y,
                last_pose_.transform.translation.z,
                last_pose_.transform.rotation.x,
                last_pose_.transform.rotation.y,
                last_pose_.transform.rotation.z,
                last_pose_.transform.rotation.w);

       
        tf2::Quaternion tf_quaternion(last_pose_.transform.rotation.x,
                                      last_pose_.transform.rotation.y,
                                      last_pose_.transform.rotation.z,
                                      last_pose_.transform.rotation.w);

        tf2::Matrix3x3 mat(tf_quaternion);
        
        mat.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(node_ptr_->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);


        geometry_msgs::msg::Pose2D pose;  
        pose.x = last_pose_.transform.translation.x;
        pose.y = last_pose_.transform.translation.y;
        pose.theta = yaw;
        request->initial_pose= pose; //geometry_msgs.msg.Pose2D(x=0.0, y=0.0, theta=0.0); // Change at run time

        future_ = client_->async_send_request(request);
        map_loading_done_flag_ = true; 
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus LoadMapFromSlam::onRunning()
    {
        if (map_loading_done_flag_)
        {   
            map_loading_done_flag_ = false;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void LoadMapFromSlam::onHalted()
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "Map loading was halted.");
    }

    geometry_msgs::msg::TransformStamped LoadMapFromSlam::getMapToBaseLink()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
 
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_ptr_->get_logger(), "Could not get transform: %s", ex.what());
        }

        return transformStamped;
    }


///////////////////////////////////////////////// Wait Event //////////////////////////////////////////////////////////

WaitEvent::WaitEvent(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
    {
        subscription_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
            "wait_event", 10,
            std::bind(&WaitEvent::wait_event_callback, this, std::placeholders::_1));
    }

    BT::PortsList WaitEvent::providedPorts()
    {
        return {BT::InputPort<std::string>("event")};
    }

    BT::NodeStatus WaitEvent::onStart()
    {
        BT::Optional<std::string> posegraph_file = getInput<std::string>("event");
        const std::string location_file = node_ptr_->get_parameter("location_file").as_string();
        YAML::Node locations = YAML::LoadFile(location_file);

        wait_event_flag_ = false; 
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus WaitEvent::onRunning()
    {
        if (wait_event_flag_)
        {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void WaitEvent::onHalted()
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "Wait event was halted.");
    }

    void WaitEvent::wait_event_callback(const std_msgs::msg::Bool & msg)
    {
        if(msg.data)
        {
            wait_event_flag_ = true;
        }
    }
                    