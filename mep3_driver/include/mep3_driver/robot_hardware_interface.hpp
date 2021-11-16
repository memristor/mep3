#ifndef ROBOT_HARDWARE_INTERFACE_HPP
#define ROBOT_HARDWARE_INTERFACE_HPP

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace mep3_driver
{
    class RobotHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
    {
    public:
        hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::return_type start() override;
        hardware_interface::return_type stop() override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read() override;
        hardware_interface::return_type write() override;

    private:
        double mLeftWheelVelocityCommand;
        double mLeftWheelPositionState;
        double mRightWheelVelocityCommand;
        double mRightWheelPositionState;
    };
}

#endif
