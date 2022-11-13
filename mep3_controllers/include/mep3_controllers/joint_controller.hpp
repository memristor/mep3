#ifndef __MEP3_CONTROLLERS__JOINT_CONTROLLER_H__
#define __MEP3_CONTROLLERS__JOINT_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>

namespace mep3_controllers
{
    class Joint
    {
    public:
        const hardware_interface::LoanedStateInterface* position_state;
        hardware_interface::LoanedCommandInterface* velocity_command;
    };

    class JointController
        : public controller_interface::ControllerInterface
    {
    public:
        JointController();
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_init() override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    protected:
        std::shared_ptr<Joint> get_joint(const std::string &name);

        std::vector<std::string> joints_;
    };
}

#endif // __MEP3_CONTROLLERS__JOINT_CONTROLLER_H__