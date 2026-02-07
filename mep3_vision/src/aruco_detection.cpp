#include <functional>
#include <memory>
#include <thread>

#include "mep3_msgs/action/aruco.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

typedef mep3_msgs::action::Aruco aruco_msg;

namespace mep3_vision
{
  
class ArucoActionServer : public rclcpp::Node
{
public:

  using GoalHandleAruco = rclcpp_action::ServerGoalHandle<aruco_msg>;

  explicit ArucoActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("aruco_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<aruco_msg>(this, "aruco", 
      std::bind(&ArucoActionServer::handle_goal, this, _1, _2),
      std::bind(&ArucoActionServer::handle_cancel, this, _1),
      std::bind(&ArucoActionServer::handle_accepted, this, _1)
    );
  }

private:
  rclcpp_action::Server<aruco_msg>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const aruco_msg::Goal> goal){
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAruco> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel gas");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAruco> goal_handle){
    using namespace std::placeholders;
    std::thread{std::bind(&ArucoActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleAruco> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Ide gas");
    /*rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<aruco_msg::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }*/
  }
};  // class ArucoActionServer

}  // namespace mep3_vision

RCLCPP_COMPONENTS_REGISTER_NODE(mep3_vision::ArucoActionServer)