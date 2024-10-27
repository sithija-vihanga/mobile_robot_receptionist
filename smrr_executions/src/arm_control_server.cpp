#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "smrr_interfaces/action/arm_control_server.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>

using namespace std::placeholders;

namespace smrr_executions
{
class ArmControlServer : public rclcpp::Node
{
public:
    explicit ArmControlServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("arm_control_server", options)
        {
            RCLCPP_INFO(rclcpp::get_logger("ArmControlServer"), "Starting the Arm Control Server...");

            action_server_ = rclcpp_action::create_server<smrr_interfaces::action::ArmControlServer>(
                this, "arm_control_server",
                std::bind(&ArmControlServer::goalCallback, this, _1, _2),
                std::bind(&ArmControlServer::cancelCallback, this, _1),
                std::bind(&ArmControlServer::acceptedCallback, this, _1)
            );

        }

private:
    rclcpp_action::Server<smrr_interfaces::action::ArmControlServer>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::vector<double> joint_group_positions_;

    rclcpp_action::GoalResponse goalCallback(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const smrr_interfaces::action::ArmControlServer::Goal> goal
    )
    {
        RCLCPP_INFO(rclcpp::get_logger("ArmControlServer"), "Target pose received");

        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<smrr_interfaces::action::ArmControlServer>> goal_handle
    )
    {
        (void)goal_handle;
        RCLCPP_INFO(rclcpp::get_logger("ArmControlServer"), "Received a request to cancel goal ");    
        
        if (move_group_interface_){
            move_group_interface_->stop();
        }

        return rclcpp_action::CancelResponse::ACCEPT;
        
    }

    void acceptedCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<smrr_interfaces::action::ArmControlServer>> goal_handle
    )
    {

        if (!move_group_interface_) {
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        }

        std::thread{std::bind(&ArmControlServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<smrr_interfaces::action::ArmControlServer>> goal_handle
    )
    {
        RCLCPP_INFO(rclcpp::get_logger("ArmControlServer"), "Received request to execute goal");

        auto result = std::make_shared<smrr_interfaces::action::ArmControlServer::Result>();

        joint_group_positions_ = goal_handle->get_goal()->target_joints_angles.data;
        move_group_interface_->setJointValueTarget(joint_group_positions_);
        move_group_interface_->move();

        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(rclcpp::get_logger("ArmControlServer"), "Goal succeeded");

    }

};
}// namespace smrr_executions

RCLCPP_COMPONENTS_REGISTER_NODE(smrr_executions::ArmControlServer)
