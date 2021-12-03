#ifndef MOTION_PROFILE_HPP_
#define MOTION_PROFILE_HPP_

#include "rclcpp/time.hpp"

class MotionProfile
{
public:
    MotionProfile();
    MotionProfile(double position_initial, double velocity_max, double acceleration_max);
    void plan(double position_initial, double setpoint, double velocity_initial, double velocity_final, rclcpp::Time time);
    double update(rclcpp::Time time);
    double get_position();
    double get_velocity();
    bool finished_;

private:
    double position_;
    double position_initial_;
    double setpoint_;
    double velocity_max_;
    double acceleration_max_;
    double velocity_initial_;
    double velocity_cruising_;
    double velocity_final_;
    double velocity_current_;
    double acceleration_;
    double deceleration_;
    double acceleration_current_;

    double t0_, t1_, t2_, t3_;
    double y1_, y2_;

    rclcpp::Time time_initial_;
};

#endif
