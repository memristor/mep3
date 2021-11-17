#include "mep3_driver/robot_hardware_interface.hpp"

#include <iostream>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <cmath>
#include <cstring>

namespace mep3_driver
{
    hardware_interface::return_type RobotHardwareInterface::configure(const hardware_interface::HardwareInfo &info)
    {
        if (configure_default(info) != hardware_interface::return_type::OK)
        {
            return hardware_interface::return_type::ERROR;
        }

        kp_left_ = std::stof(info_.hardware_parameters["kp_left"]);
        ki_left_ = std::stof(info_.hardware_parameters["ki_left"]);
        kd_left_ = std::stof(info_.hardware_parameters["kd_left"]);

        kp_right_ = std::stof(info_.hardware_parameters["kp_right"]);
        ki_right_ = std::stof(info_.hardware_parameters["ki_right"]);
        kd_right_ = std::stof(info_.hardware_parameters["kd_right"]);

        std::cout << "KP Left: " << kp_left_ << "\tKI Left: " << ki_left_ << "\tKD Left: " << kd_left_ << std::endl;
        std::cout << "KP Left: " << kp_right_ << "\tKI_Right: " << ki_right_ << "\tKD Right: " << kd_right_ << std::endl;


        status_ = hardware_interface::status::CONFIGURED;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotHardwareInterface::start()
    {
        status_ = hardware_interface::status::STARTED;

        // init variables
        mLeftWheelVelocityCommand = 0;
        mLeftWheelPositionState = 0;
        mRightWheelVelocityCommand = 0;
        mRightWheelPositionState = 0;

        mPrevLeftWheelRaw = 0;
        mPrevRightWheelRaw = 0;
        mOdomLeftOverflow = 0;
        mOdomRightOverflow = 0;

        motion_board_.init();
        motion_board_.start();

        motion_board_.set_kp_left(kp_left_);
        motion_board_.set_ki_left(ki_left_);
        motion_board_.set_kd_left(kd_left_);

        motion_board_.set_kp_right(kp_right_);
        motion_board_.set_ki_right(ki_right_);
        motion_board_.set_kd_right(kd_right_);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotHardwareInterface::stop()
    {
        status_ = hardware_interface::status::STOPPED;
        motion_board_.halt();

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
        int32_t tmp_left, tmp_right;
        std::tie(tmp_left, tmp_right) = motion_board_.get_encoders();

        const int32_t leftWheelRaw = tmp_left;
        const int32_t rightWheelRaw = tmp_right;

        // Handle overflow
        if (llabs((int64_t)mPrevLeftWheelRaw - (int64_t)leftWheelRaw) > POW2(31) - 1)
            mOdomLeftOverflow = (mPrevLeftWheelRaw > 0 && leftWheelRaw < 0) ? mOdomLeftOverflow + 1 : mOdomLeftOverflow - 1;
        if (llabs((int64_t)mPrevRightWheelRaw - (int64_t)rightWheelRaw) > POW2(31) - 1)
            mOdomRightOverflow = (mPrevRightWheelRaw > 0 && rightWheelRaw < 0) ? mOdomRightOverflow + 1 : mOdomRightOverflow - 1;
        const int64_t leftWheelCorrected = mOdomLeftOverflow * POW2(32) + leftWheelRaw;
        const int64_t rightWheelCorrected = mOdomRightOverflow * POW2(32) + rightWheelRaw;
        const double leftWheelRad = leftWheelCorrected / (ENCODER_RESOLUTION / (2 * M_PI));
        const double rightWheelRad = rightWheelCorrected / (ENCODER_RESOLUTION / (2 * M_PI));

        mLeftWheelPositionState = leftWheelRad;
        mRightWheelPositionState = rightWheelRad;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RobotHardwareInterface::write()
    {
        // Send left and right wheel velocity commands to the robot

        /*std::cout << "mLeftWheelVelocityCommand: " << mLeftWheelVelocityCommand << std::endl;
        std::cout << "mRightWheelVelocityCommand: " << mRightWheelVelocityCommand << std::endl;*/

        // convert rad/s to inc/2ms         -> NOTE: 2 ms!! Control loop on motion board runs at 500 Hz = 2 ms period
        const double speed_double_left = mLeftWheelVelocityCommand * 8.192 / M_PI;
        const double speed_double_right = mRightWheelVelocityCommand * 8.192 / M_PI;

        const int16_t speed_increments_left = (int16_t) round(speed_double_left);
        const int16_t speed_increments_right = (int16_t) round(speed_double_right);

        motion_board_.set_setpoints(speed_increments_left, speed_increments_right);

        /*std::cout << "mLeftWheelVelocityCommand: " << mLeftWheelVelocityCommand << std::endl;
        std::cout << "mRightWheelVelocityCommand: " << mRightWheelVelocityCommand << std::endl;

        std::cout << "speed_increments_left: " << speed_increments_left << std::endl;
        std::cout << "speed_increments_right: " << speed_increments_right << std::endl;*/

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mep3_driver::RobotHardwareInterface, hardware_interface::SystemInterface)
