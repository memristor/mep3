#ifndef ROBOT_HARDWARE_INTERFACE_HPP
#define ROBOT_HARDWARE_INTERFACE_HPP

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "motion_board_driver.hpp"

#define POW2(N) (1UL << (N))

namespace mep3_driver
{
class RobotHardwareInterface
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

private:
  double left_wheel_velocity_command_;
  double left_wheel_position_state_;
  double left_wheel_velocity_state_;
  double right_wheel_velocity_command_;
  double right_wheel_position_state_;
  double right_wheel_velocity_state_;

  int32_t prev_left_wheel_raw_;
  int32_t prev_right_wheel_raw_;
  int odom_left_overflow_;
  int odom_right_overflow_;

  float kp_left_, ki_left_, kd_left_;
  float kp_right_, ki_right_, kd_right_;
  double update_rate_;

  static constexpr double ENCODER_RESOLUTION = 8192.0;  // 2048 * 4

  MotionBoardDriver motion_board_;
};
}  // namespace mep3_driver

#endif
