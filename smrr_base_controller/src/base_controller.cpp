#include "smrr_base_controller/base_controller.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace smrr_base_controller
{
BaseController::BaseController()
{
}


BaseController::~BaseController()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BaseController"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn BaseController::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("BaseController"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.reserve(2);
  position_commands_.reserve(4);
  position_states_.reserve(6);
  velocity_states_.reserve(2);
  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BaseController::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

 
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "shoulder_r_joint", hardware_interface::HW_IF_POSITION, &position_states_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "bicep_r_joint", hardware_interface::HW_IF_POSITION, &position_states_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "elbow_r_joint", hardware_interface::HW_IF_POSITION, &position_states_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "wrist_r_joint", hardware_interface::HW_IF_POSITION, &position_states_[3]));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_wheel_joint", hardware_interface::HW_IF_POSITION, &position_states_[4]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_wheel_joint", hardware_interface::HW_IF_POSITION, &position_states_[5]));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &velocity_states_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &velocity_states_[1]));


  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> BaseController::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "shoulder_r_joint", hardware_interface::HW_IF_POSITION, &position_commands_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "bicep_r_joint", hardware_interface::HW_IF_POSITION, &position_commands_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "elbow_r_joint", hardware_interface::HW_IF_POSITION, &position_commands_[3]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "wrist_r_joint", hardware_interface::HW_IF_POSITION, &position_commands_[4]));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[1]));
  

  return command_interfaces;
}


CallbackReturn BaseController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Starting robot hardware ...");

  // Reset commands and states
  velocity_commands_ = { 0.0, 0.0};
  position_commands_ = { 0.0, 0.0, 0.0, 0.0};
  position_states_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  velocity_states_ = { 0.0, 0.0};

  try
  { 
    bool isConnected = arduino_.IsOpen();
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), 
            "Port connection status: %s", isConnected ? "true" : "false");
    RCLCPP_INFO(rclcpp::get_logger("BaseController"),
              "Opening port");
    arduino_.Open(port_);
    isConnected = arduino_.IsOpen();
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), 
            "Port connection status: %s", isConnected ? "true" : "false");
    RCLCPP_INFO(rclcpp::get_logger("BaseController"),
              "Port opened");
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    RCLCPP_INFO(rclcpp::get_logger("BaseController"),
              "baudrate assigned");
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("BaseController"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("BaseController"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn BaseController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BaseController"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BaseController::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  if (arduino_.IsOpen())
  {
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Reading");
    // Interpret the string
    if(arduino_.IsDataAvailable())
    {
      auto dt = (rclcpp::Clock().now() - last_run_).seconds();
      std::string message;
      arduino_.ReadLine(message);
      std::stringstream ss(message);
      std::string res;
      int multiplier = 1;
      while(std::getline(ss, res, ','))
      {
        multiplier = res.at(1) == 'p' ? 1 : -1;

        if(res.at(0) == 'r')
        {
          velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
          position_states_.at(0) += velocity_states_.at(0) * dt;
        }
        else if(res.at(0) == 'l')
        {
          velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
          position_states_.at(1) += velocity_states_.at(1) * dt;
        }
      }
      last_run_ = rclcpp::Clock().now();
    }
  }
  
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type BaseController::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  if (arduino_.IsOpen())
  {
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Writing");
    // Implement communication protocol with the Arduino
    std::stringstream message_stream;
    char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
    char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
    std::string compensate_zeros_right = "";
    std::string compensate_zeros_left = "";
    if(std::abs(velocity_commands_.at(0)) < 10.0)
    {
      compensate_zeros_right = "0";
    }
    else
    {
      compensate_zeros_right = "";
    }
    if(std::abs(velocity_commands_.at(1)) < 10.0)
    {
      compensate_zeros_left = "0";
    }
    else
    {
      compensate_zeros_left = "";
    }
    
    message_stream << std::fixed << std::setprecision(2) << 
      "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) << 
      ",l" <<  left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";

    try
    {
      arduino_.Write(message_stream.str());
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("BaseController"),
                          "Something went wrong while sending the message "
                              << message_stream.str() << " to the port " << port_);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}
}  // namespace base_controller
PLUGINLIB_EXPORT_CLASS(smrr_base_controller::BaseController, hardware_interface::SystemInterface)