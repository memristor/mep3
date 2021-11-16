#include "mep3_driver/robot_hardware_interface.hpp"

#include <iostream>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mep3_driver
{
    hardware_interface::return_type RobotHardwareInterface::configure(const hardware_interface::HardwareInfo &info)
    {
        if (configure_default(info) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }
        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotHardwareInterface::start()
    {
        status_ = hardware_interface::status::STARTED;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotHardwareInterface::stop()
    {
        status_ = hardware_interface::status::STOPPED;
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;
        interfaces.emplace_back(hardware_interface::StateInterface("left_motor", hardware_interface::HW_IF_POSITION, &mLeftWheelPositionState));
        interfaces.emplace_back(hardware_interface::StateInterface("right_motor", hardware_interface::HW_IF_POSITION, &mRightWheelPositionState));
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RobotHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;
        interfaces.emplace_back(hardware_interface::CommandInterface("left_motor", hardware_interface::HW_IF_VELOCITY, &mLeftWheelVelocityCommand));
        interfaces.emplace_back(hardware_interface::CommandInterface("right_motor", hardware_interface::HW_IF_VELOCITY, &mRightWheelVelocityCommand));
        return interfaces;
    }

    hardware_interface::return_type RobotHardwareInterface::read()
    {
        // Read encoder data from the robot
        mLeftWheelPositionState = 0;
        mRightWheelPositionState = 0;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotHardwareInterface::write()
    {
        // Send left and right wheel velocity commands to the robot
        std::cout << "mLeftWheelVelocityCommand: " << mLeftWheelVelocityCommand << std::endl;
        std::cout << "mRightWheelVelocityCommand: " << mRightWheelVelocityCommand << std::endl;

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mep3_driver::RobotHardwareInterface, hardware_interface::SystemInterface)
