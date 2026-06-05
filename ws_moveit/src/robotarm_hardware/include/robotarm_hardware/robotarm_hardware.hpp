#pragma once

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <std_msgs/msg/int32_multi_array.hpp>

#include <vector>
#include <string>

namespace robotarm_hardware
{

class RobotArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotArmHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Estados y comandos de los joints (rad / m)
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;

  // Para la lógica tipo bridge (en grados)
  std::vector<double> last_sent_deg_;    // últimos grados enviados (rev1..rev4)
  bool first_write_ = true;
  double min_angle_change_deg_ = 1.0;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("RobotArmHardware")};

  // Nodo interno y publisher a /motor_angles
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_pub_;
};

}  // namespace robotarm_hardware

