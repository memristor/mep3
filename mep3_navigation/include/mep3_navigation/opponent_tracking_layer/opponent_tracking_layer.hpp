#ifndef MEP3_NAVIGATION__OPPONENT_TRACKING_LAYER_HPP_
#define MEP3_NAVIGATION__OPPONENT_TRACKING_LAYER_HPP_

#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mep3_navigation {
class OpponentTrackingLayer : public rclcpp::Node {
public:
  OpponentTrackingLayer();
  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

protected:
  void set_occupancy_header(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr opponent_pub_;

  int8_t costmap_threshold_;
  int8_t occupied_value_;
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> occupancy_;
  bool occupancy_initialized_;
};

} // namespace mep3_navigation

#endif // MEP3_NAVIGATION__OPPONENT_TRACKING_LAYER_HPP_
