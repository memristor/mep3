#ifndef MEP3_NAVIGATION__MOVE_HPP_
#define MEP3_NAVIGATION__MOVE_HPP_

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_util/execution_timer.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "mep3_msgs/msg/move_command.hpp"
#include "mep3_msgs/msg/move_properties.hpp"
#include "mep3_msgs/msg/move_state.hpp"
#include "mep3_msgs/action/move.hpp"
#include "ruckig/ruckig.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mep3_navigation
{
  class Move : public rclcpp::Node
  {
  public:
    Move(std::string name);
    void init();
    void update();
    int get_update_rate() { return update_rate_; };

  private:
    void on_action();
    void on_command_received(const mep3_msgs::msg::MoveCommand::SharedPtr msg);

    bool init_move(const mep3_msgs::msg::MoveCommand::SharedPtr msg);
    void init_rotation(double diff_yaw);
    void regulate_rotation(geometry_msgs::msg::Twist *cmd_vel, double diff_yaw);
    void init_translation(double diff_x, double diff_y);
    void regulate_translation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y);

    void state_rotating_towards_goal(const tf2::Transform &tf_base_target, const tf2::Transform &tf_odom_base, geometry_msgs::msg::Twist *cmd_vel);
    void state_translating(const tf2::Transform &tf_base_target, geometry_msgs::msg::Twist *cmd_vel);
    void state_rotating_at_goal(const tf2::Transform &tf_base_target, geometry_msgs::msg::Twist *cmd_vel);
    void state_stopping();

    double get_diff_heading(const tf2::Transform &tf_base_target);
    double get_diff_final_orientation(const tf2::Transform &tf_base_target);
    double get_distance(const tf2::Transform &tf_base_target);

    bool update_odom_target_tf();

    void stop_robot();
    void update_state_msg(tf2::Transform &tf_base_target);

    int update_rate_;
    double stopping_distance_;

    mep3_msgs::msg::MoveState state_msg_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<mep3_msgs::msg::MoveCommand>::SharedPtr command_sub_;
    rclcpp::Publisher<mep3_msgs::msg::MoveState>::SharedPtr state_pub_;
    std::shared_ptr<nav2_util::SimpleActionServer<mep3_msgs::action::Move>> action_server_;

    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_;

    mep3_msgs::msg::MoveCommand::SharedPtr command_;
    mep3_msgs::msg::MoveCommand::SharedPtr default_command_{new mep3_msgs::msg::MoveCommand()};

    std::string robot_frame_;
    std::string odom_frame_;

    tf2::Transform tf_odom_target_;
    rclcpp::Time end_time_;
    bool target_updated_{false};
    double last_error_x_;
    double last_error_y_;
    double last_error_yaw_;

    rclcpp::Time debouncing_end_;
    rclcpp::Duration debouncing_duration_{0, 0};
    void debouncing_reset();

    double angular_stuck_coeff_;
    double linear_stuck_coeff_;

    ruckig::Ruckig<1> *rotation_ruckig_{nullptr};
    ruckig::InputParameter<1> rotation_ruckig_input_;
    ruckig::OutputParameter<1> rotation_ruckig_output_;
    double previous_yaw_;
    int multiturn_n_;
    bool use_multiturn_;

    rclcpp::Time start_action_time_;

    tf2::Transform locked_tf_odom_base_;
    bool lock_tf_odom_base_{false};

    ruckig::Ruckig<1> *translation_ruckig_{nullptr};
    ruckig::InputParameter<1> translation_ruckig_input_;
    ruckig::OutputParameter<1> translation_ruckig_output_;

    rclcpp::Duration command_timeout_{0, 0};
    rclcpp::Time last_command_received_;

    uint8_t state_{mep3_msgs::msg::MoveState::STATE_IDLE};
    uint8_t previous_state_{mep3_msgs::msg::MoveState::STATE_IDLE};

    // TODO: Remove
    double transform_tolerance_{0.5};
  };

}

#endif // MEP3_NAVIGATION__MOVE_HPP_
