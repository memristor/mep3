#ifndef __MEP3_CONTROLLERS__PUMP_CONTROLLER_H__
#define __MEP3_CONTROLLERS__PUMP_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include "nav2_util/simple_action_server.hpp"
#include "mep3_msgs/action/vacuum_pump_command.hpp"

namespace mep3_controllers
{
    struct Pump
    {
        Pump(){};
        std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> pump_command_handle;
        std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> valve_command_handle;
        std::string name;
        bool connect;

        std::shared_ptr<nav2_util::SimpleActionServer<mep3_msgs::action::VacuumPumpCommand>> action_server;
        bool active;
	double disconnect_start_time;
    };

    class PumpController
        : public controller_interface::ControllerInterface
    {
    public:
        PumpController();
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
        std::vector<std::shared_ptr<Pump>> pumps_;
        void on_action_called(std::shared_ptr<Pump> pump);
    };
}

#endif // __MEP3_CONTROLLERS__PUMP_CONTROLLER_H__
