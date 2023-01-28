#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mep3_controllers/joint_position_controller.hpp"

namespace mep3_controllers
{
    JointPositionController::JointPositionController() {}

    void JointPositionController::on_action_called(std::shared_ptr<Joint> joint)
    {
        RCLCPP_WARN(get_node()->get_logger(), "Action execute!");
        auto goal = joint->action_server->get_current_goal();

        double max_velocity = 1.0;
        if (goal->max_velocity != 0)
            max_velocity = goal->max_velocity;

        joint->target_position = goal->position;
        joint->max_velocity = max_velocity;
        joint->active = true;

        while (rclcpp::ok() && joint->active)
            ;
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
            std::shared_ptr<Joint> joint = std::make_shared<Joint>();
            joint->name = joint_name;
            joint->action_server = std::make_shared<nav2_util::SimpleActionServer<mep3_msgs::action::JointPositionCommand>>(
                get_node(),
                ("joint_position_command/" + joint->name).c_str(),
                std::bind(&JointPositionController::on_action_called, this, joint),
                nullptr,
                std::chrono::milliseconds(1500),
                true);
            joint->action_server->activate();
            joint->active = false;
            joints_.emplace_back(joint);
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration JointPositionController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (std::shared_ptr<Joint> joint : joints_)
        {
            command_interfaces_config.names.push_back(joint->name + "/position");
            command_interfaces_config.names.push_back(joint->name + "/velocity");
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
        // ros2 action send_goal /big/joint_position_command/arm_joint mep3_msgs/action/JointPositionCommand "{ position: -1.57 }"

        for (std::shared_ptr<Joint> joint : joints_)
        {
            if (joint->active)
            {
                RCLCPP_WARN(get_node()->get_logger(), "%s is moving to %lf", joint->name.c_str(), joint->target_position);
                joint->position_handle->get().set_value(joint->target_position);
                joint->velocity_handle->get().set_value(joint->max_velocity);

                // Return the result
                auto result = std::make_shared<mep3_msgs::action::JointPositionCommand::Result>();
                result->set__result(0);
                joint->action_server->succeeded_current(result);
                joint->active = false;
            }
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn JointPositionController::on_activate(const rclcpp_lifecycle::State &)
    {
        for (std::shared_ptr<Joint> joint : joints_)
        {
            const auto position_command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&joint](const auto &interface)
                {
                    return interface.get_prefix_name() == joint->name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });
            if (position_command_handle == command_interfaces_.end())
            {
                return controller_interface::CallbackReturn::FAILURE;
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", joint->name.c_str());
            }
            joint->position_handle = std::ref(*position_command_handle);

            const auto velocity_command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&joint](const auto &interface)
                {
                    return interface.get_prefix_name() == joint->name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });
            if (velocity_command_handle == command_interfaces_.end())
            {
                return controller_interface::CallbackReturn::FAILURE;
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", joint->name.c_str());
            }
            joint->velocity_handle = std::ref(*velocity_command_handle);
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