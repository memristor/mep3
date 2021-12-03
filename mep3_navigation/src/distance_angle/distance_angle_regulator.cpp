#include "mep3_navigation/distance_angle/distance_angle_regulator.hpp"
#include <iostream>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <string>

DistanceAngleRegulator::DistanceAngleRegulator() : Node("distance_angle_regulator")
{
    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 100, std::bind(&DistanceAngleRegulator::odometry_callback, this, _1));
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    command_subscription_ = this->create_subscription<mep3_msgs::msg::MotionCommand>("/da_command", 100, std::bind(&DistanceAngleRegulator::command_callback, this, _1));

    this->declare_parameter("kp_distance", 25000.0);
    this->declare_parameter("ki_distance", 1.5);
    this->declare_parameter("kd_distance", 1120.0);

    this->declare_parameter("kp_angle", 40000.0);
    this->declare_parameter("ki_angle", 22.0);
    this->declare_parameter("kd_angle", 1500.0);

    debug_ = this->declare_parameter("debug", true);
    parameters_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DistanceAngleRegulator::parameters_callback, this, std::placeholders::_1));

    this->get_parameter("kp_distance", regulator_distance_.kp);
    this->get_parameter("ki_distance", regulator_distance_.ki);
    this->get_parameter("kd_distance", regulator_distance_.kd);
    regulator_distance_.clamp_min = -2000.0;
    regulator_distance_.clamp_max = 2000.0;
    regulator_distance_.integrator_min = -800.0;
    regulator_distance_.integrator_max = 800.0;

    this->get_parameter("kp_angle", regulator_angle_.kp);
    this->get_parameter("ki_angle", regulator_angle_.ki);
    this->get_parameter("kd_angle", regulator_angle_.kd);
    regulator_angle_.clamp_min = -5000.0;
    regulator_angle_.clamp_max = 5000.0;
    regulator_angle_.integrator_min = -1500.0;
    regulator_angle_.integrator_max = 1500.0;

    robot_distance_ = 0;
    position_initialized_ = false;

    distance_profile_ = MotionProfile(0, 0.1, 0.02);
    angle_profile_ = MotionProfile(0, 0.5, 0.25);
}

void DistanceAngleRegulator::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    rclcpp::Time time = this->get_clock()->now();

    double robot_x = msg->pose.pose.position.x;
    double robot_y = msg->pose.pose.position.y;

    if (!position_initialized_)
    {
        prev_robot_x_ = robot_x;
        prev_robot_y_ = robot_y;
        position_initialized_ = true;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        robot_angle_ = yaw;

        distance_profile_.plan(0, 0, 0, 0, time);
        angle_profile_.plan(robot_angle_, robot_angle_, 0, 0, time);
    }

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot_angle_ = yaw;
    double delta_x = robot_x - prev_robot_x_;
    double delta_y = robot_y - prev_robot_y_;
    double distance_increment = std::hypot(delta_x, delta_y);
    double distance_increment_angle = std::atan2(delta_y, delta_x);

    int sign = 1;
    if (std::abs(robot_angle_ - distance_increment_angle) > 0.1)
        sign = -1;

    robot_distance_ += sign * distance_increment;

    regulator_distance_.feedback = robot_distance_;
    regulator_angle_.feedback = robot_angle_;

    regulator_distance_.reference = distance_profile_.update(time);
    // we need to wrap the angle from -PI to PI
    double angle_ref = angle_normalize(angle_profile_.update(time));
    double normalized_angle_error = angle_normalize(angle_ref - robot_angle_);
    regulator_angle_.reference = regulator_angle_.feedback + normalized_angle_error;

    pid_regulator_update(&regulator_distance_);
    pid_regulator_update(&regulator_angle_);

    if (debug_)
    {
        std::cout << "Robot distance: " << robot_distance_ << std::endl;
        std::cout << "Regulator distance reference: " << regulator_distance_.reference << std::endl;
        std::cout << "Regulator distance error: " << regulator_distance_.error << std::endl;
        std::cout << std::endl;

        std::cout << "Robot angle: " << robot_angle_ << std::endl;
        std::cout << "Robot angle deg: " << robot_angle_ * 180 / M_PI << std::endl;
        std::cout << "Robot angle reference deg: " << regulator_angle_.reference * 180 / M_PI << std::endl;
        std::cout << "Regulator angle error deg: " << regulator_angle_.error * 180 / M_PI << std::endl;
        std::cout << std::endl;
    }

    geometry_msgs::msg::Twist motor_command;
    motor_command.linear.x = regulator_distance_.command / 10000.0;
    motor_command.angular.z = regulator_angle_.command / 10000.0;
    twist_publisher_->publish(motor_command);

    prev_robot_x_ = robot_x;
    prev_robot_y_ = robot_y;
}

rcl_interfaces::msg::SetParametersResult DistanceAngleRegulator::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : parameters)
    {
        if (param.get_name() == "kp_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            regulator_distance_.kp = param.as_double();
        }
        else if (param.get_name() == "ki_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            regulator_distance_.ki = param.as_double();
        }
        else if (param.get_name() == "kd_distance" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            regulator_distance_.kd = param.as_double();
        }
        else if (param.get_name() == "kp_angle" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            regulator_angle_.kp = param.as_double();
        }
        else if (param.get_name() == "ki_angle" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            regulator_angle_.ki = param.as_double();
        }
        else if (param.get_name() == "kd_angle" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
            regulator_angle_.kd = param.as_double();
        }
        else if (param.get_name() == "debug" && param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        {
            debug_ = param.as_bool();
        }
    }

    return result;
}

void DistanceAngleRegulator::command_callback(const mep3_msgs::msg::MotionCommand::SharedPtr msg)
{
    if (msg->command == "forward")
    {
        forward(msg->value);
    }
    else if (msg->command == "rotate_absolute")
    {
        rotate_absolute(msg->value);
    }
    else if (msg->command == "rotate_relative")
    {
        rotate_relative(msg->value);
    }
}

double DistanceAngleRegulator::angle_normalize(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }

    return angle;
}
void DistanceAngleRegulator::forward(double distance)
{
    rclcpp::Time time = this->get_clock()->now();
    distance_profile_.plan(distance_profile_.get_position(), distance_profile_.get_position() + distance, distance_profile_.get_velocity(), 0, time);
}

void DistanceAngleRegulator::rotate_absolute(double angle)
{
    angle = angle_normalize(angle);
    rclcpp::Time time = this->get_clock()->now();
    angle_profile_.plan(robot_angle_, angle, angle_profile_.get_velocity(), 0, time);
}

void DistanceAngleRegulator::rotate_relative(double angle)
{
    rclcpp::Time time = this->get_clock()->now();
    angle_profile_.plan(angle_profile_.get_position(), angle_profile_.get_position() + angle, angle_profile_.get_velocity(), 0, time);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceAngleRegulator>());
    rclcpp::shutdown();

    return 0;
}
