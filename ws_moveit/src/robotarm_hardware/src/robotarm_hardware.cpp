#include "robotarm_hardware/robotarm_hardware.hpp"

#include <cmath>

namespace robotarm_hardware
{

hardware_interface::CallbackReturn RobotArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t num_joints = info_.joints.size();

  hw_positions_.assign(num_joints, 0.0);
  hw_commands_.assign(num_joints, 0.0);
  last_sent_deg_.assign(4, 0.0);  // rev1..rev4

  // Leer parámetros del xacro (si los pones)
  auto it = info_.hardware_parameters.find("min_angle_change_deg");
  if (it != info_.hardware_parameters.end())
  {
    min_angle_change_deg_ = std::stod(it->second);
  }

  RCLCPP_INFO(
    logger_,
    "RobotArmHardware inicializado con %zu joints. min_angle_change_deg = %.2f",
    num_joints, min_angle_change_deg_);

  // Nodo interno y publisher a /motor_angles
  node_ = rclcpp::Node::make_shared("robotarm_hardware_node");
  motor_pub_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>(
    "/motor_angles", rclcpp::QoS(10));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RobotArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint_name = info_.joints[i].name;

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_VELOCITY, nullptr));  // opcional
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint_name = info_.joints[i].name;

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RobotArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Al activar, ponemos posiciones = comandos
  hw_positions_ = hw_commands_;
  first_write_ = true;

  RCLCPP_INFO(logger_, "RobotArmHardware activado");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "RobotArmHardware desactivado");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Simulamos lectura desde hardware: comandos -> posiciones
hardware_interface::return_type RobotArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  hw_positions_ = hw_commands_;
  return hardware_interface::return_type::OK;
}

// Lógica tipo bridge:
//   rev1..rev3 -> delta en grados (si supera min_angle_change_deg_)
//   rev4       -> absoluto en grados
// Se publica en /motor_angles (Int32MultiArray [4])
hardware_interface::return_type RobotArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const std::size_t num = info_.joints.size();
  if (num < 4)
  {
    // esperamos al menos 4 juntas (rev1..rev4)
    return hardware_interface::return_type::OK;
  }

  // Se asume que los 4 primeros joints son rev1..rev4 y están en hw_commands_[0..3]
  std::vector<double> current_deg(4, 0.0);
  for (std::size_t i = 0; i < 4; ++i)
  {
    current_deg[i] = hw_commands_[i] * 180.0 / M_PI;
  }

  if (first_write_)
  {
    last_sent_deg_ = current_deg;
    first_write_ = false;
    return hardware_interface::return_type::OK;
  }

  int motor_cmd[4] = {0, 0, 0, 0};
  bool send_any = false;

  // rev1..rev3: delta
  for (std::size_t i = 0; i < 3; ++i)
  {
    double delta = current_deg[i] - last_sent_deg_[i];
    if (std::abs(delta) >= min_angle_change_deg_)
    {
      motor_cmd[i] = static_cast<int>(std::round(delta));
      send_any = true;
    }
  }

  // rev4: absoluto
  motor_cmd[3] = static_cast<int>(std::round(current_deg[3]));
  send_any = true;

  // Actualizar último enviado
  last_sent_deg_ = current_deg;

  if (send_any)
  {
    if (motor_pub_ && motor_pub_->get_subscription_count() > 0)
    {
      // Publicar en /motor_angles
      std_msgs::msg::Int32MultiArray msg;
      msg.data.resize(4);
      msg.data[0] = motor_cmd[0];
      msg.data[1] = motor_cmd[1];
      msg.data[2] = motor_cmd[2];
      msg.data[3] = motor_cmd[3];

      motor_pub_->publish(msg);
    }
    else
    {
      // Sin subscriptores: solo log
      RCLCPP_INFO(
        logger_,
        "RobotArmHardware (sin subs /motor_angles) -> motor_cmd [Δr1,Δr2,Δr3,abs_r4] = [%d, %d, %d, %d]",
        motor_cmd[0], motor_cmd[1], motor_cmd[2], motor_cmd[3]);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace robotarm_hardware

// Registrar plugin
PLUGINLIB_EXPORT_CLASS(
  robotarm_hardware::RobotArmHardware,
  hardware_interface::SystemInterface)
