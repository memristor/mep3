#include "mep3_driver/robot_hardware_interface.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mep3_driver
{
hardware_interface::return_type RobotHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  kp_left_ = std::stof(info_.hardware_parameters["kp_left"]);
  ki_left_ = std::stof(info_.hardware_parameters["ki_left"]);
  kd_left_ = std::stof(info_.hardware_parameters["kd_left"]);

  kp_right_ = std::stof(info_.hardware_parameters["kp_right"]);
  ki_right_ = std::stof(info_.hardware_parameters["ki_right"]);
  kd_right_ = std::stof(info_.hardware_parameters["kd_right"]);

  update_rate_ = std::stod(info_.hardware_parameters["update_rate"]);

  std::cout << "KP Left: " << kp_left_ << "\tKI Left: " << ki_left_ << "\tKD Left: " << kd_left_
            << std::endl;
  std::cout << "KP Left: " << kp_right_ << "\tKI_Right: " << ki_right_
            << "\tKD Right: " << kd_right_ << std::endl;

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardwareInterface::start()
{
  status_ = hardware_interface::status::STARTED;

  // init variables
  left_wheel_velocity_command_ = 0;
  left_wheel_position_state_ = 0;
  right_wheel_velocity_command_ = 0;
  right_wheel_position_state_ = 0;

  prev_left_wheel_raw_ = 0;
  prev_right_wheel_raw_ = 0;
  odom_left_overflow_ = 0;
  odom_right_overflow_ = 0;

  int board_init_status = motion_board_.init();
  if (board_init_status != 0) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mep3_driver"),
      "Motion board low lever driver init failed! Is 'can0' up?\n");
    return hardware_interface::return_type::ERROR;
  }
  motion_board_.start();

  motion_board_.set_kp_left(kp_left_);
  motion_board_.set_ki_left(ki_left_);
  motion_board_.set_kd_left(kd_left_);

  motion_board_.set_kp_right(kp_right_);
  motion_board_.set_ki_right(ki_right_);
  motion_board_.set_kd_right(kd_right_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardwareInterface::stop()
{
  status_ = hardware_interface::status::STOPPED;
  motion_board_.halt();

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(hardware_interface::StateInterface(
    "left_motor", hardware_interface::HW_IF_POSITION, &left_wheel_position_state_));
  interfaces.emplace_back(hardware_interface::StateInterface(
    "left_motor", hardware_interface::HW_IF_VELOCITY, &left_wheel_velocity_state_));
  interfaces.emplace_back(hardware_interface::StateInterface(
    "right_motor", hardware_interface::HW_IF_POSITION, &right_wheel_position_state_));
  interfaces.emplace_back(hardware_interface::StateInterface(
    "right_motor", hardware_interface::HW_IF_VELOCITY, &right_wheel_velocity_state_));
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(hardware_interface::CommandInterface(
    "left_motor", hardware_interface::HW_IF_VELOCITY, &left_wheel_velocity_command_));
  interfaces.emplace_back(hardware_interface::CommandInterface(
    "right_motor", hardware_interface::HW_IF_VELOCITY, &right_wheel_velocity_command_));
  return interfaces;
}

hardware_interface::return_type RobotHardwareInterface::read()
{
  // Read encoder data from the robot
  int32_t tmp_left, tmp_right;
  std::tie(tmp_left, tmp_right) = motion_board_.get_encoders();

  const int32_t left_wheel_raw = tmp_left;
  const int32_t right_wheel_raw = tmp_right;

  // Handle overflow
  const int64_t prev_left_wheel_corrected = odom_left_overflow_ * POW2(32) + prev_left_wheel_raw_;
  const int64_t prev_right_wheel_corrected =
    odom_right_overflow_ * POW2(32) + prev_right_wheel_raw_;
  if (llabs((int64_t)prev_left_wheel_raw_ - (int64_t)left_wheel_raw) > POW2(31) - 1)
    odom_left_overflow_ = (prev_left_wheel_raw_ > 0 && left_wheel_raw < 0)
                            ? odom_left_overflow_ + 1
                            : odom_left_overflow_ - 1;
  if (llabs((int64_t)prev_right_wheel_raw_ - (int64_t)right_wheel_raw) > POW2(31) - 1)
    odom_right_overflow_ = (prev_right_wheel_raw_ > 0 && right_wheel_raw < 0)
                             ? odom_right_overflow_ + 1
                             : odom_right_overflow_ - 1;
  const int64_t left_wheel_corrected = odom_left_overflow_ * POW2(32) + left_wheel_raw;
  const int64_t right_wheel_corrected = odom_right_overflow_ * POW2(32) + right_wheel_raw;
  const double left_wheel_rad = left_wheel_corrected / (ENCODER_RESOLUTION / (2 * M_PI));
  const double right_wheel_rad = right_wheel_corrected / (ENCODER_RESOLUTION / (2 * M_PI));
  const double prev_left_wheel_rad = prev_left_wheel_corrected / (ENCODER_RESOLUTION / (2 * M_PI));
  const double prev_right_wheel_rad =
    prev_right_wheel_corrected / (ENCODER_RESOLUTION / (2 * M_PI));

  // calculate velocities
  const double left_wheel_velocity_rad =
    (left_wheel_rad - prev_left_wheel_rad) * update_rate_;  // 1 / sample_period = update_rate
  const double right_wheel_velocity_rad = (right_wheel_rad - prev_right_wheel_rad) * update_rate_;

  left_wheel_position_state_ = left_wheel_rad;
  right_wheel_position_state_ = right_wheel_rad;

  left_wheel_velocity_state_ = left_wheel_velocity_rad;
  right_wheel_velocity_state_ = right_wheel_velocity_rad;

  prev_left_wheel_raw_ = left_wheel_raw;
  prev_right_wheel_raw_ = right_wheel_raw;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardwareInterface::write()
{
  // Send left and right wheel velocity commands to the robot

  // convert rad/s to inc/2ms         -> NOTE: 2 ms!! Control loop on motion board runs at 500 Hz = 2 ms period
  const double speed_double_left = left_wheel_velocity_command_ * 8.192 / M_PI;
  const double speed_double_right = right_wheel_velocity_command_ * 8.192 / M_PI;

  // RCLCPP_INFO(rclcpp::get_logger("mep3_driver"), "Setpoint left rad/s: %lf", left_wheel_velocity_command_);
  // RCLCPP_INFO(rclcpp::get_logger("mep3_driver"), "Setpoint right rad/s: %lf\n", right_wheel_velocity_command_);

  const int16_t speed_increments_left = (int16_t)round(speed_double_left);
  const int16_t speed_increments_right = (int16_t)round(speed_double_right);

  motion_board_.set_setpoints(speed_increments_left, speed_increments_right);

  return hardware_interface::return_type::OK;
}
}  // namespace mep3_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mep3_driver::RobotHardwareInterface, hardware_interface::SystemInterface)
