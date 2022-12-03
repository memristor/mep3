// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MEP3_NAVIGATION__TEMPORAL_OBSTACLE_LAYER_HPP_
#define MEP3_NAVIGATION__TEMPORAL_OBSTACLE_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "mep3_msgs/msg/temporal_obstacle.hpp"

namespace mep3_navigation {
class TemporalObstacleLayer : public nav2_costmap_2d::Layer {
public:
  TemporalObstacleLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double *min_x, double *min_y, double *max_x,
                            double *max_y);
  virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i,
                           int min_j, int max_i, int max_j);

  virtual void reset() { return; }

  virtual void onFootprintChanged();

  virtual bool isClearable() { return false; }

private:
  rclcpp::Subscription<mep3_msgs::msg::TemporalObstacle>::SharedPtr
      add_obstacle_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      remove_obstacle_subscriber_;

  void on_new_obstacle(const mep3_msgs::msg::TemporalObstacle::SharedPtr msg);
  void on_remove_obstacle(const std_msgs::msg::String::SharedPtr msg);

  std::string add_obstacle_topic_;
  std::string remove_obstacle_topic_;

  bool need_update_;
  std::vector<mep3_msgs::msg::TemporalObstacle::SharedPtr> obstacles_;
};

} // namespace mep3_navigation

#endif // MEP3_NAVIGATION__TEMPORAL_OBSTACLE_LAYER_HPP_
