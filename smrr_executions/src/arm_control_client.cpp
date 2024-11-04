#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>
#include "smrr_interfaces/action/arm_control_server.hpp"
#include "smrr_interfaces/srv/arm_control.hpp"
#include "std_msgs/msg/bool.hpp"

#include <memory>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace smrr_executions
{
class ArmControlClient : public rclcpp::Node
{
public :
    explicit ArmControlClient(const rclcpp::NodeOptions& options) 
        : Node("arm_control_client", options)
        {   
            
            yaml_path_  = "/home/sithija/mobile_receptionist_ws/src/smrr_elevator_behavior/config/elevator_interaction.yaml" ;

            client_     = rclcpp_action::create_client<smrr_interfaces::action::ArmControlServer>(this, "arm_control_server");
            
            publisher_  = this->create_publisher<std_msgs::msg::Bool>("wait_event", 10);
            
            service     = this->create_service<smrr_interfaces::srv::ArmControl>("start_arm_motion", std::bind(&ArmControlClient::ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
        }

private:
    rclcpp_action::Client<smrr_interfaces::action::ArmControlServer>::SharedPtr client_;

    std::string yaml_path_ ;
    int current_goal_stage_ = 0;

    rclcpp::Service<smrr_interfaces::srv::ArmControl>::SharedPtr service;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

    void ServiceCallback(const std::shared_ptr<smrr_interfaces::srv::ArmControl::Request> request,
          std::shared_ptr<smrr_interfaces::srv::ArmControl::Response> response)
    {   
        if (!client_->wait_for_action_server(5s)){
            RCLCPP_ERROR(rclcpp::get_logger("ArmControlClient"), "Action server not available, cancelling the action client...");
            rclcpp::shutdown();
            return;
        }

        sendNextGoal();
    }

    void sendNextGoal()
    {
        auto send_goal_options = rclcpp_action::Client<smrr_interfaces::action::ArmControlServer>::SendGoalOptions();
        send_goal_options.goal_response_callback  = std::bind(&ArmControlClient::goalCallback, this, _1);
        send_goal_options.result_callback         = std::bind(&ArmControlClient::resultCallback, this, _1);

        // Determine which goal to send based on the current stage
        auto goal_msg = smrr_interfaces::action::ArmControlServer::Goal();
        
        if (current_goal_stage_ == 0){
            goal_msg.target_joints_angles = read_yaml("initial_joint_angles");
        }
        else if (current_goal_stage_ == 1){
            goal_msg.target_joints_angles = read_yaml("target_joint_angles");
        }
        else if (current_goal_stage_ == 2){
            goal_msg.target_joints_angles = read_yaml("end_joint_angles");
        }
        else if (current_goal_stage_ == 3){
            goal_msg.target_joints_angles = read_yaml("home_joint_angles");
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("ArmControlClient"), "All goals completed.");
            auto message = std_msgs::msg::Bool();
            message.data = true;
            publisher_->publish(message);
            RCLCPP_INFO(rclcpp::get_logger("ArmControlClient"), "Sent multinav resume cmd");
            current_goal_stage_ = 0;
            return;
        }

        // Send the goal asynchronously
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goalCallback(const rclcpp_action::ClientGoalHandle<smrr_interfaces::action::ArmControlServer>::SharedPtr& goal_handle)
    {
        if (!goal_handle){
            RCLCPP_ERROR(rclcpp::get_logger("ArmControlClient"), "Goal was rejected by server");
            }
        else{
            RCLCPP_INFO(rclcpp::get_logger("ArmControlClient"), "Goal accepted by server, waiting for result");
            }
    }



    void resultCallback(const rclcpp_action::ClientGoalHandle<smrr_interfaces::action::ArmControlServer>::WrappedResult& result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(rclcpp::get_logger("ArmControlClient"), "Goal %d reached successfully", (current_goal_stage_+1));
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("ArmControlClient"), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(rclcpp::get_logger("ArmControlClient"), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("ArmControlClient"), "Unknown result code");
            return;
        }

        current_goal_stage_++;
        sendNextGoal();
    }


    std_msgs::msg::Float64MultiArray read_yaml(std::string target_joint_angles)
    {
        YAML::Node data;
        try 
        {
            data = YAML::LoadFile(this->yaml_path_);
        }
        catch (const YAML::Exception& e) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("ArmControlClient"), "Failed to load YAML file: %s", e.what());
            rclcpp::shutdown();
        }

        double joint1 = data["elevator_interaction"][target_joint_angles]["joint1"].as<double>();
        double joint2 = data["elevator_interaction"][target_joint_angles]["joint2"].as<double>();
        double joint3 = data["elevator_interaction"][target_joint_angles]["joint3"].as<double>();
        double joint4 = data["elevator_interaction"][target_joint_angles]["joint4"].as<double>();

        std_msgs::msg::Float64MultiArray array ;
        array.data = {joint1, joint2, joint3, joint4};
        return array ;
    }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(smrr_executions::ArmControlClient);