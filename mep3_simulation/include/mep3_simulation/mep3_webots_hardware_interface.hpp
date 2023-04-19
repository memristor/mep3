#ifndef MEP3_SIMULATION__MEP3_WEBOTS_HARDWARE_INTERFACE_HPP_
#define MEP3_SIMULATION__MEP3_WEBOTS_HARDWARE_INTERFACE_HPP_

#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "webots_ros2_control/Ros2ControlSystemInterface.hpp"

namespace mep3_simulation
{
  struct Pin {
    double output;
    double previous_output;
    std::string name;
  };

  struct FakeJoint {
    double position;
    double velocity;
    double effort;
    double command_position;
    double command_velocity;
    double command_effort;
    std::string name;
    uint64_t write_counter;
  };

  class Mep3WebotsHardwareInterface : public webots_ros2_control::Ros2ControlSystemInterface
  {
  public:
    void init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  private:
    webots_ros2_driver::WebotsNode *node_;
    std::vector<Pin> pins_;
    std::vector<FakeJoint> fake_joints_;
  };
} // namespace mep3_simulation

#endif // MEP3_SIMULATION__MEP3_WEBOTS_HARDWARE_INTERFACE_HPP_
