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

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace mep3_navigation
{

  TemporalObstacleLayer::TemporalObstacleLayer()
      : last_min_x_(-std::numeric_limits<float>::max()),
        last_min_y_(-std::numeric_limits<float>::max()),
        last_max_x_(std::numeric_limits<float>::max()),
        last_max_y_(std::numeric_limits<float>::max())
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

    add_obstacle_subscriber_ = node->create_subscription<mep3_msgs::msg::TemporalObstacle>(
        add_obstacle_topic_, rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&TemporalObstacleLayer::on_new_obstacle, this, std::placeholders::_1));

    remove_obstacle_subscriber_ = node->create_subscription<diagnostic_msgs::msg::KeyValue>(
        remove_obstacle_topic_, rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&TemporalObstacleLayer::on_remove_obstacle, this, std::placeholders::_1));

    need_recalculation_ = false;
    current_ = true;
  }

  void TemporalObstacleLayer::on_new_obstacle(const mep3_msgs::msg::TemporalObstacle::SharedPtr msg)
  {
  }

  void TemporalObstacleLayer::on_remove_obstacle(const diagnostic_msgs::msg::KeyValue::SharedPtr msg)
  {
  }

  void
  TemporalObstacleLayer::updateBounds(
      double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
      double *min_y, double *max_x, double *max_y)
  {
    if (need_recalculation_)
    {
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      // For some reason when I make these -<double>::max() it does not
      // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
      // -<float>::max() instead.
      *min_x = -std::numeric_limits<float>::max();
      *min_y = -std::numeric_limits<float>::max();
      *max_x = std::numeric_limits<float>::max();
      *max_y = std::numeric_limits<float>::max();
      need_recalculation_ = false;
    }
    else
    {
      double tmp_min_x = last_min_x_;
      double tmp_min_y = last_min_y_;
      double tmp_max_x = last_max_x_;
      double tmp_max_y = last_max_y_;
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      *min_x = std::min(tmp_min_x, *min_x);
      *min_y = std::min(tmp_min_y, *min_y);
      *max_x = std::max(tmp_max_x, *max_x);
      *max_y = std::max(tmp_max_y, *max_y);
    }
  }

  void
  TemporalObstacleLayer::onFootprintChanged()
  {
  }

  void
  TemporalObstacleLayer::updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
      int max_i,
      int max_j)
  {
    if (!enabled_)
    {
      return;
    }

    // master_array - is a direct pointer to the resulting master_grid.
    // master_grid - is a resulting costmap combined from all layers.
    // By using this pointer all layers will be overwritten!
    // To work with costmap layer and merge it with other costmap layers,
    // please use costmap_ pointer instead (this is pointer to current
    // costmap layer grid) and then call one of updates methods:
    // - updateWithAddition()
    // - updateWithMax()
    // - updateWithOverwrite()
    // - updateWithTrueOverwrite()
    // In this case using master_array pointer is equal to modifying local costmap_
    // pointer and then calling updateWithTrueOverwrite():
    unsigned char *master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

    // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
    // These variables are used to update the costmap only within this window
    // avoiding the updates of whole area.
    //
    // Fixing window coordinates with map size if necessary.
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);

    // Simply computing one-by-one cost per each cell
    int gradient_index;
    for (int j = min_j; j < max_j; j++)
    {
      // Reset gradient_index each time when reaching the end of re-calculated window
      // by OY axis.
      gradient_index = 0;
      for (int i = min_i; i < max_i; i++)
      {
        int index = master_grid.getIndex(i, j);
        // setting the gradient cost
        unsigned char cost = (LETHAL_OBSTACLE - gradient_index * GRADIENT_FACTOR) % 255;
        if (gradient_index <= GRADIENT_SIZE)
        {
          gradient_index++;
        }
        else
        {
          gradient_index = 0;
        }
        master_array[index] = cost;
      }
    }
  }

} // namespace mep3_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mep3_navigation::TemporalObstacleLayer, nav2_costmap_2d::Layer)
