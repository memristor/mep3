#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mep3_controllers/joint_position_controller.hpp"

namespace mep3_controllers
{
    JointPositionController::JointPositionController() {}

    void JointPositionController::on_action_called(std::shared_ptr<Joint> joint)
    {
        auto goal = joint->action_server->get_current_goal();
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Motor %s action called to position %lf (velocity %lf, tolerance %lf)",
            joint->name.c_str(),
            goal->position,
            goal->max_velocity,
            goal->tolerance);

        double max_velocity = 1.0;
        if (goal->max_velocity != 0)
            max_velocity = goal->max_velocity;

        double tolerance = 99999;
        if (goal->tolerance != 0)
            tolerance = goal->tolerance;

        joint->target_position = goal->position;
        joint->max_velocity = max_velocity;
        joint->tolerance = tolerance;
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
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during on_init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn JointPositionController::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Configure JointPositionController");

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
        std::vector<std::string> conf_names;
        for (std::shared_ptr<Joint> joint : joints_)
        {
            conf_names.push_back(joint->name + "/position");
            conf_names.push_back(joint->name + "/velocity");
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration JointPositionController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (std::shared_ptr<Joint> joint : joints_)
        {
            conf_names.push_back(joint->name + "/position");
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::return_type JointPositionController::update(const rclcpp::Time &time, const rclcpp::Duration & /* period */)
    {
        // ros2 action send_goal /big/joint_position_command/m6 mep3_msgs/action/JointPositionCommand "{ position: -1.57 }"

        for (std::shared_ptr<Joint> joint : joints_)
        {
            if (joint->active)
            {
                joint->position_command_handle->get().set_value(joint->target_position);
                joint->velocity_command_handle->get().set_value(joint->max_velocity);

                // Return the result
                if (fabs(joint->position_handle->get().get_value() - joint->target_position) < joint->tolerance)
                {
                    auto result = std::make_shared<mep3_msgs::action::JointPositionCommand::Result>();
                    result->set__result(0);
                    joint->action_server->succeeded_current(result);
                    joint->active = false;
                }
                else if (joint->action_server->is_cancel_requested())
                {
                    auto result = std::make_shared<mep3_msgs::action::JointPositionCommand::Result>();
                    result->set__result(0);
                    joint->active = false;
                }
                else if (joint->action_server->is_preempt_requested())
                {
                    auto result = std::make_shared<mep3_msgs::action::JointPositionCommand::Result>();
                    result->set__result(1);
                    joint->active = false;
                }
            }
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn JointPositionController::on_activate(const rclcpp_lifecycle::State &)
    {
        for (std::shared_ptr<Joint> joint : joints_)
        {
            // Position command
            const auto position_command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&joint](const auto &interface)
                {
                    return interface.get_prefix_name() == joint->name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });
            if (position_command_handle == command_interfaces_.end())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", joint->name.c_str());
                return controller_interface::CallbackReturn::FAILURE;
            }
            joint->position_command_handle = std::ref(*position_command_handle);

            // Velocity command
            const auto velocity_command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&joint](const auto &interface)
                {
                    return interface.get_prefix_name() == joint->name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });
            if (velocity_command_handle == command_interfaces_.end())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint command handle for %s", joint->name.c_str());
                return controller_interface::CallbackReturn::FAILURE;
            }
            joint->velocity_command_handle = std::ref(*velocity_command_handle);

            // Position state
            const auto position_handle = std::find_if(
                state_interfaces_.begin(), state_interfaces_.end(),
                [&joint](const auto &interface)
                {
                    return interface.get_prefix_name() == joint->name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });
            if (position_handle == state_interfaces_.end())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain joint state handle for %s", joint->name.c_str());
                return controller_interface::CallbackReturn::FAILURE;
            }
            joint->position_handle = std::ref(*position_handle);
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