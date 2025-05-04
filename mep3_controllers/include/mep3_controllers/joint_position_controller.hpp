#ifndef __MEP3_CONTROLLERS__JOINT_CONTROLLER_H__
#define __MEP3_CONTROLLERS__JOINT_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include "nav2_util/simple_action_server.hpp"
#include "mep3_msgs/action/joint_position_command.hpp"

namespace mep3_controllers
{
    struct Joint
    {
        Joint(){};
        std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> position_command_handle;
        std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> velocity_command_handle;
        std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> multiturn_command_handle;
        std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_handle;
        std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort_handle;
        std::string name;

        uint64_t start_time_ns;
        std::shared_ptr<nav2_util::SimpleActionServer<mep3_msgs::action::JointPositionCommand>> action_server;

        double target_position;
        double max_velocity;
        double tolerance;
        double timeout;
        double max_effort;
        bool active;
        double command_mode;
    };

    class JointPositionController
        : public controller_interface::ControllerInterface
    {
    public:
        JointPositionController();
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
        std::vector<std::shared_ptr<Joint>> joints_;
        void on_action_called(std::shared_ptr<Joint> joint);
    };
}

#endif // __MEP3_CONTROLLERS__JOINT_CONTROLLER_H__