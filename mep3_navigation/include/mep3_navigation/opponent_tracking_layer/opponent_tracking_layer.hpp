#ifndef MEP3_NAVIGATION__OPPONENT_TRACKING_LAYER_HPP_
#define MEP3_NAVIGATION__OPPONENT_TRACKING_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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

  std::shared_ptr<nav_msgs::msg::OccupancyGrid> occupancy_;
  bool occupancy_initialized_;
};

} // namespace mep3_navigation

#endif // MEP3_NAVIGATION__OPPONENT_TRACKING_LAYER_HPP_
