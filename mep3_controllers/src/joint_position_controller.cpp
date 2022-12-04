#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mep3_controllers/joint_position_controller.hpp"

namespace mep3_controllers
{
    JointPositionController::JointPositionController() {}

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

        joints_ = get_node()->get_parameter("joints").as_string_array();
        if (joints_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "The 'joints' parameter is empty");
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration JointPositionController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto &joint : joints_)
        {
            command_interfaces_config.names.push_back(joint + "/position");
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
        for (std::string &joint_name : joints_)
        {
            auto joint = get_joint(joint_name);
            if (!joint)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found.", joint_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
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

    std::shared_ptr<Joint> JointPositionController::get_joint(const std::string &name)
    {
        const auto position_command = std::find_if(command_interfaces_.cbegin(), command_interfaces_.cend(), [&name](const hardware_interface::LoanedCommandInterface &interface)
                                                 { return interface.get_prefix_name() == name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION; });
        if (position_command == command_interfaces_.cend())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "%s position state interface not found", name.c_str());
            return nullptr;
        }
        auto joint = std::make_shared<Joint>();
        joint->position_command = &(*position_command);
        return joint;
    }
} // namespace mep3_controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    mep3_controllers::JointPositionController, controller_interface::ControllerInterface)