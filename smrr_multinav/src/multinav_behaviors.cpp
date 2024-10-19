// Author: Sithija Ranaraja

#include "multinav_behaviors.h"

///////////////////////////////////////////////// Go to Pose //////////////////////////////////////////////////////////

GoToPose::GoToPose(const std::string &name,
                  const BT::NodeConfiguration &config,
                  rclcpp::Node::SharedPtr node_ptr)
        : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
    {
        action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
        done_flag_ = false;
        RCLCPP_INFO(node_ptr_->get_logger(), "Go to pose initialized");
    }

    BT::PortsList GoToPose::providedPorts()
    {
        return {BT::InputPort<std::string>("loc")};
    }

    BT::NodeStatus GoToPose::onStart()
    {   
        RCLCPP_INFO(node_ptr_->get_logger(), "Go to pose started");
        BT::Optional<std::string> loc = getInput<std::string>("loc");
        const std::string multinav_config = node_ptr_->get_parameter("multinav_config").as_string();

        YAML::Node multinav = YAML::LoadFile(multinav_config);
        std::vector<float> pose = multinav["elevator_locations"][loc.value()].as<std::vector<float>>();

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
        RCLCPP_INFO(node_ptr_->get_logger(), "Go to pose running");
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
        RCLCPP_INFO(node_ptr_->get_logger(), "Map loading initialized");
    }

    BT::PortsList LoadMapFromSlam::providedPorts()
    {
        return {BT::InputPort<std::string>("posegraph_file")};
    }

    BT::NodeStatus LoadMapFromSlam::onStart()
    {   
        RCLCPP_INFO(node_ptr_->get_logger(), "Map loading started");
        BT::Optional<std::string> posegraph_file = getInput<std::string>("posegraph_file");
        const std::string multinav_config = node_ptr_->get_parameter("multinav_config").as_string();
        YAML::Node multinav = YAML::LoadFile(multinav_config);
        std::string map_path = multinav["preloaded_maps"][posegraph_file.value()].as<std::string>();

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
        RCLCPP_INFO(node_ptr_->get_logger(), "Map loading running");
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
        RCLCPP_INFO(node_ptr_->get_logger(), "Wait event initialized");
    }

    BT::PortsList WaitEvent::providedPorts()
    {
        return {BT::InputPort<std::string>("event")};
    }

    BT::NodeStatus WaitEvent::onStart()
    {   
        RCLCPP_INFO(node_ptr_->get_logger(), "Wait event started");
        BT::Optional<std::string> posegraph_file = getInput<std::string>("event");
        const std::string multinav_config = node_ptr_->get_parameter("multinav_config").as_string();
        YAML::Node multinav = YAML::LoadFile(multinav_config);

        wait_event_flag_ = false; 
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus WaitEvent::onRunning()
    {   
        RCLCPP_INFO(node_ptr_->get_logger(), "Wait event running");
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

///////////////////////////////////////////////// Rotate to Elevator //////////////////////////////////////////////////////////

ElevatorLoading::ElevatorLoading(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr) ,A(Eigen::MatrixXf::Ones(20, 2)), B(Eigen::VectorXf::Zero(20))
    {   
        orientation_publisher_  = node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>("diff_drive_controller/cmd_vel", 10);
        laser_subscription_     = node_ptr_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ElevatorLoading::laser_callback, this, _1));    
        const std::string multinav_config = node_ptr_->get_parameter("multinav_config").as_string();
        YAML::Node multinav = YAML::LoadFile(multinav_config);
        K_P = multinav["rotate_to_elevator"]["K_P"].as<float>();
        K_D = multinav["rotate_to_elevator"]["K_D"].as<float>();

        RCLCPP_INFO(node_ptr_->get_logger(), "Elevator loading initialized");
    }

    BT::PortsList ElevatorLoading::providedPorts()
    {
        return {BT::InputPort<std::string>("type")};
    }

    BT::NodeStatus ElevatorLoading::onStart()
    {   
        RCLCPP_INFO(node_ptr_->get_logger(), "Elevator loading started");
        action_type = getInput<std::string>("type");
        complete_flag_ = false;
        timer_ = node_ptr_->create_wall_timer(500ms, std::bind(&ElevatorLoading::scan_extractor, this));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus ElevatorLoading::onRunning()
    {   
        RCLCPP_INFO(node_ptr_->get_logger(), "Elevator loading running");
        if (complete_flag_)
        {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void ElevatorLoading::onHalted()
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "rotate to elevator was halted.");
    }

    void ElevatorLoading::laser_callback(const sensor_msgs::msg::LaserScan & msg)
    {
        laser_scan = msg;
    }

    void ElevatorLoading::scan_extractor()
    {   
        if (laser_scan.ranges.size() < 250) {
            RCLCPP_WARN(node_ptr_->get_logger(), "Not enough laser scan data received.");
            return;
        }

        std::vector<float> laser_slice(laser_scan.ranges.begin() + 149, laser_scan.ranges.begin() + 250);
        if (laser_slice.size() < 100) { 
            RCLCPP_WARN(node_ptr_->get_logger(), "Laser slice does not contain enough elements.");
            return;
        }

        filtered_points.clear();
        for (int i = 0; i < 20; i++) 
        {
            if (5 * i + 4 < laser_slice.size()) {
                std::sort(laser_slice.begin() + 5 * i, laser_slice.begin() + 5 * i + 5); // Sort dynamic window of 5 elements
                filtered_points.push_back(laser_slice[5 * i + 2]); // Get the middle reading as median
                A(i, 1) = laser_slice[5 * i + 2] * cos((90-(-28.8 + 3 * i)) * (M_PI / 180.0));
                B(i) = laser_slice[5 * i + 2] * sin((90-(-28.8 + 3 * i)) * (M_PI / 180.0));
            }
        }
        if(action_type.value() == "rotate")
        {
            Eigen::VectorXf X = ((A.transpose()*A).inverse())*A.transpose()*B;
            current_angle = (X[1]);
            if(current_angle<0.04 and current_angle>-0.04)
            {
                omega = 0;
                complete_flag_ = true;
                timer_.reset();
                RCLCPP_INFO(node_ptr_->get_logger(),"Rotation Complete");
                
            }
            else
            {
                omega = -(K_P*current_angle - K_D*(current_angle - prev_angle));
            }
            auto cmd_vel = geometry_msgs::msg::TwistStamped();
            cmd_vel.twist.angular.z = omega;
            orientation_publisher_->publish(cmd_vel);

            prev_angle = current_angle;
        }
        else if(action_type.value() == "check_door")
        {
            laser_mean = std::accumulate(laser_slice.begin(), laser_slice.end(), 0.0)/laser_slice.size();
            RCLCPP_INFO(node_ptr_->get_logger(),"laser mean: %f",laser_mean);
            if(laser_mean > 3.0)
            {
                complete_flag_ = true;
                timer_.reset();
                RCLCPP_INFO(node_ptr_->get_logger(),"Door opened");
            }
        }
        else if(action_type.value() == "wait")
        {
            std::this_thread::sleep_for(std::chrono::seconds(10));
            complete_flag_ = true;
            timer_.reset();
            RCLCPP_INFO(node_ptr_->get_logger(),"Wait complete");
        }
            //RCLCPP_INFO(node_ptr_->get_logger(),"laser mean: %f",laser_mean);
            // std::cout<< "Laser mean :" <<laser_mean<<std::endl;
    
    }
    