#include "mep3_navigation/stuck_behavior/stuck_behavior.hpp"
#include "pluginlib/class_list_macros.hpp"

mep3_navigation::StuckBehavior::StuckBehavior() :
  nav2_behaviors::TimedBehavior<ActionT>()
{
  std::cout << "STUCK BEHAVIOR CONSTRUCTOR" << std::endl << std::endl;
  RCLCPP_WARN(this->logger_, "STUCK BEHAVIOR CONSTRUCTOR!");
}

mep3_navigation::StuckBehavior::~StuckBehavior() = default;

nav2_behaviors::Status mep3_navigation::StuckBehavior::onRun(const std::shared_ptr<const typename ActionT::Goal> command)
{
  RCLCPP_INFO(this->logger_, "onRun successful!");
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status mep3_navigation::StuckBehavior::onCycleUpdate()
{
  RCLCPP_INFO(this->logger_, "onCycleUpdate successful!");
  return nav2_behaviors::Status::RUNNING;
}

PLUGINLIB_EXPORT_CLASS(mep3_navigation::StuckBehavior, nav2_core::Behavior)

