#include "mep3_vision/object_detection.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace mep3_vision
{
  typedef mep3_msgs::action::Camera CameraAction;
  using GoalHandleCamera = rclcpp_action::ServerGoalHandle<CameraAction>;
  static std::mutex buffer_mux;

  ObjectDetection::ObjectDetection(const rclcpp::NodeOptions &options) : Node("object_detection", options)
  {
    subscription_ = this->create_subscription<table_msg>("table_state", 5, [this](const table_msg::SharedPtr msg){this->callback(msg);});

    action_server_ = rclcpp_action::create_server<CameraAction>(
      this,
      "camera",
      std::bind(&ObjectDetection::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ObjectDetection::handle_cancel, this, std::placeholders::_1),
      std::bind(&ObjectDetection::handle_accepted, this, std::placeholders::_1)
  );

  }

  void ObjectDetection::callback(const table_msg::SharedPtr message){
    std::unique_lock<std::mutex> ul(buffer_mux);
    for(int i = 0; i < 12; i++){
      buffer[i] = (uint8_t)message->data[i];
    }
    ul.unlock();
  }

  rclcpp_action::GoalResponse ObjectDetection::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CameraAction::Goal> goal){
    (void)uuid;
    group_select_ = goal->group_select;

    if(group_select_ > 17){
      RCLCPP_ERROR(this->get_logger(), "Invalid value for group_select. Must be above 0 and below 18");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse ObjectDetection::handle_cancel(const std::shared_ptr<GoalHandleCamera> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel camera detection");
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void ObjectDetection::handle_accepted(const std::shared_ptr<GoalHandleCamera> goal_handle){
    using namespace std::placeholders;
    std::thread{std::bind(&ObjectDetection::execute, this, _1), goal_handle}.detach();
  }

  void ObjectDetection::execute(const std::shared_ptr<GoalHandleCamera> goal_handle){
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<mep3_msgs::action::Camera::Result>();
    uint8_t local_result = 0;
    result->response = 0;
    
    std::unique_lock<std::mutex> ul(buffer_mux);
    local_result = buffer[group_select_];
    ul.unlock();

    result->response = local_result;

    std::cout << (int)local_result << std::endl;
    RCLCPP_INFO(this->get_logger(), "Group info: %d", (int)local_result);
    goal_handle->succeed(result);
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(mep3_vision::ObjectDetection)
