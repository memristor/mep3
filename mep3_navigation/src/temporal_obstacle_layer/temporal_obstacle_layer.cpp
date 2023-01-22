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

#include "mep3_navigation/temporal_obstacle_layer/temporal_obstacle_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav2_costmap_2d/array_parser.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace mep3_navigation
{

  // ros2 topic pub -1 /big/add_obstacle mep3_msgs/msg/TemporalObstacle "{label: 'test', polygon: [{x: 0.0, y: 0.0}, {x: -0.2, y: 0.0}, {x: 0.0, y: 0.2}]}"
  // ros2 topic pub -1 /big/remove_obstacle std_msgs/msg/String "{data: 'test'}"

  TemporalObstacleLayer::TemporalObstacleLayer()
  {
  }

  void TemporalObstacleLayer::onInitialize()
  {
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    declareParameter("add_obstacle_topic", rclcpp::ParameterValue("add_obstacle"));
    node->get_parameter(name_ + "." + "add_obstacle_topic", add_obstacle_topic_);

    declareParameter("remove_obstacle_topic", rclcpp::ParameterValue("remove_obstacle"));
    node->get_parameter(name_ + "." + "remove_obstacle_topic", remove_obstacle_topic_);

    std::vector<std::string> predefined_obstacle_labels;
    declareParameter("predefined_obstacle_labels", rclcpp::ParameterValue(std::vector<std::string>()));
    node->get_parameter(name_ + "." + "predefined_obstacle_labels", predefined_obstacle_labels);

    for (std::string &obstacle_label : predefined_obstacle_labels)
    {
      std::string obstacle_raw;
      std::string error;
      declareParameter(obstacle_label, rclcpp::ParameterValue("[]"));
      node->get_parameter(name_ + "." + obstacle_label, obstacle_raw);
      std::vector<std::vector<float>> point_pairs = nav2_costmap_2d::parseVVF(obstacle_raw, error);
      mep3_msgs::msg::TemporalObstacle::SharedPtr obstacle_message = std::make_shared<mep3_msgs::msg::TemporalObstacle>();
      obstacle_message->label = obstacle_label;
      for (std::vector<float> &point_pair : point_pairs)
      {
        geometry_msgs::msg::Point point;
        point.x = point_pair[0];
        point.y = point_pair[1];
        obstacle_message->polygon.push_back(point);
        obstacles_.push_back(obstacle_message);
      }
    }

    add_obstacle_subscriber_ = node->create_subscription<mep3_msgs::msg::TemporalObstacle>(
        add_obstacle_topic_, rclcpp::QoS(rclcpp::QoS(1).reliable()),
        std::bind(&TemporalObstacleLayer::on_new_obstacle, this, std::placeholders::_1));

    remove_obstacle_subscriber_ = node->create_subscription<std_msgs::msg::String>(
        remove_obstacle_topic_, rclcpp::QoS(rclcpp::QoS(1).reliable()),
        std::bind(&TemporalObstacleLayer::on_remove_obstacle, this, std::placeholders::_1));
  }

  void TemporalObstacleLayer::on_new_obstacle(const mep3_msgs::msg::TemporalObstacle::SharedPtr msg)
  {
    obstacles_.push_back(msg);
  }

  void TemporalObstacleLayer::on_remove_obstacle(const std_msgs::msg::String::SharedPtr msg)
  {
    std::vector<mep3_msgs::msg::TemporalObstacle::SharedPtr> new_obstacle_list;
    for (auto obstacle : obstacles_)
    {
      if (obstacle->label != msg->data)
      {
        new_obstacle_list.push_back(obstacle);
      }
    }
    obstacles_ = new_obstacle_list;
  }

  void
  TemporalObstacleLayer::updateBounds(
      double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * /*min_x*/,
      double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
  {
  }

  void
  TemporalObstacleLayer::onFootprintChanged()
  {
  }

  void
  TemporalObstacleLayer::updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid, int /* min_i */, int /* min_j */,
      int /*max_i*/,
      int /*max_j*/)
  {
    if (!enabled_)
    {
      return;
    }

    for (auto obstacle : obstacles_)
    {
      master_grid.setConvexPolygonCost(obstacle->polygon, LETHAL_OBSTACLE);
    }
  }

} // namespace mep3_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mep3_navigation::TemporalObstacleLayer, nav2_costmap_2d::Layer)
