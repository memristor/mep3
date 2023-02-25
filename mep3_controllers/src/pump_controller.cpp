#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mep3_controllers/pump_controller.hpp"

namespace mep3_controllers
{
    PumpController::PumpController() {}

    void PumpController::on_action_called(std::shared_ptr<Pump> pump)
    {
        auto goal = pump->action_server->get_current_goal();
        RCLCPP_INFO(
            get_node()->get_logger(),
            "Pump %s action called to %d",
            pump->name.c_str(),
            goal->connect);

        pump->connect = goal->connect;
        pump->active = true;
	pump->disconnect_start_time = get_node()->get_clock()->now().seconds();

        while (rclcpp::ok() && pump->active)
            ;
    }

    controller_interface::CallbackReturn PumpController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("pumps", std::vector<std::string>());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during on_init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PumpController::on_configure(const rclcpp_lifecycle::State & /* previous_state */)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Configure PumpController");

        std::vector<std::string> pump_names = get_node()->get_parameter("pumps").as_string_array();
        if (pump_names.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "The 'pumps' parameter is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        for (std::string &pump_name : pump_names)
        {
            std::shared_ptr<Pump> pump = std::make_shared<Pump>();
            pump->name = pump_name;
            pump->action_server = std::make_shared<nav2_util::SimpleActionServer<mep3_msgs::action::VacuumPumpCommand>>(
                get_node(),
                ("pump/" + pump->name).c_str(),
                std::bind(&PumpController::on_action_called, this, pump),
                nullptr,
                std::chrono::milliseconds(1500),
                true);
            pump->action_server->activate();
            pump->active = false;
            pumps_.emplace_back(pump);
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PumpController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (std::shared_ptr<Pump> pump : pumps_)
        {
            conf_names.push_back(pump->name + "_pump" + "/output");
            conf_names.push_back(pump->name + "_valve" + "/output");
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration PumpController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::return_type PumpController::update(const rclcpp::Time &time, const rclcpp::Duration & /* period */)
    {
        // ros2 action send_goal /big/pump/pump1 mep3_msgs/action/VacuumPumpCommand "{ connect: 1 }"

        for (std::shared_ptr<Pump> pump : pumps_)
        {
            if (pump->active)
            {
		bool done = false;
                if (pump->connect) {
                    pump->pump_command_handle->get().set_value(1);
                    pump->valve_command_handle->get().set_value(0);
		    done = true;
                } else {
                    pump->pump_command_handle->get().set_value(0);

		    if ((get_node()->get_clock()->now().seconds() - pump->disconnect_start_time) > 0.5) {
			    pump->valve_command_handle->get().set_value(0);
			    done = true;
		    } else
			    pump->valve_command_handle->get().set_value(1);
                }

		if (done) {
                    auto result = std::make_shared<mep3_msgs::action::VacuumPumpCommand::Result>();
                    result->set__result(0);
                    pump->action_server->succeeded_current(result);
		    pump->active = false;
		}
            }
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn PumpController::on_activate(const rclcpp_lifecycle::State &)
    {
        for (std::shared_ptr<Pump> pump : pumps_)
        {
            // Pump command
            const auto pump_command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&pump](const auto &interface)
                {
                    return interface.get_prefix_name() == pump->name + "_pump" &&
                           interface.get_interface_name() == "output";
                });
            if (pump_command_handle == command_interfaces_.end())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain pump handle for %s", pump->name.c_str());
                return controller_interface::CallbackReturn::FAILURE;
            }
            pump->pump_command_handle = std::ref(*pump_command_handle);

            // Valve command
            const auto valve_command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&pump](const auto &interface)
                {
                    return interface.get_prefix_name() == pump->name + "_valve" &&
                           interface.get_interface_name() == "output";
                });
            if (valve_command_handle == command_interfaces_.end())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to obtain valve handle for %s", pump->name.c_str());
                return controller_interface::CallbackReturn::FAILURE;
            }
            pump->valve_command_handle = std::ref(*valve_command_handle);
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PumpController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PumpController::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PumpController::on_error(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PumpController::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

} // namespace mep3_controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    mep3_controllers::PumpController, controller_interface::ControllerInterface)
