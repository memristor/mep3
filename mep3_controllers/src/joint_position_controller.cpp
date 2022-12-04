#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mep3_controllers/joint_position_controller.hpp"

namespace mep3_controllers
{
    JointPositionController::JointPositionController() {}

    void JointPositionController::on_action_called(Joint& joint) {

    }

    controller_interface::CallbackReturn JointPositionController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during on_init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Configure JointbotDriverController");

        std::vector<std::string> joint_names = get_node()->get_parameter("joints").as_string_array();
        if (joint_names.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "The 'joints' parameter is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        for (std::string &joint_name : joint_names)
        {
            Joint joint;
            joint.name = joint_name;
            joint.action_server = std::make_shared<nav2_util::SimpleActionServer<mep3_msgs::action::JointPositionCommand>>(
                get_node(),
                ("joint_position_command/" + joint.name).c_str(),
                std::bind(&JointPositionController::on_action_called, this, joint),
                nullptr,
                std::chrono::milliseconds(1500),
                true,
                rcl_action_server_get_default_options()
            );
            joint.action_server->activate();
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration JointPositionController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (Joint joint : joints_)
        {
            command_interfaces_config.names.push_back(joint.name + "/position");
        }

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration JointPositionController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        return state_interfaces_config;
    }

    controller_interface::return_type JointPositionController::update(const rclcpp::Time &time, const rclcpp::Duration & /* period */)
    {
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn JointPositionController::on_activate(const rclcpp_lifecycle::State &)
    {
        for (Joint &joint : joints_)
        {
            const auto position_command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&joint](const auto &interface)
                {
                    return interface.get_prefix_name() == joint.name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });
            if (position_command_handle == command_interfaces_.end())
            {
                return controller_interface::CallbackReturn::FAILURE;
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", joint.name.c_str());
            }
            joint.position = std::ref(*position_command_handle);
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_error(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

} // namespace mep3_controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    mep3_controllers::JointPositionController, controller_interface::ControllerInterface)