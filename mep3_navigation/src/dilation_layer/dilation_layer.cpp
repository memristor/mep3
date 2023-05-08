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

#include "mep3_navigation/dilation_layer/dilation_layer.hpp"


namespace mep3_navigation
{
  DilationLayer::DilationLayer()
  {
  }

  void DilationLayer::onInitialize()
  {
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    declareParameter("size", rclcpp::ParameterValue(0.15));
    node->get_parameter(name_ + "." + "size", size_);

    // MORPH_RECT: 0
    // MORPH_CROSS: 1
    // MORPH_ELLIPSE: 2
    declareParameter("type", rclcpp::ParameterValue(2));
    node->get_parameter(name_ + "." + "type", type_);

    current_ = true;
  }

  void
  DilationLayer::updateBounds(
      double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * /*min_x*/,
      double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
  {
  }

  void
  DilationLayer::onFootprintChanged()
  {
  }

  void
  DilationLayer::updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid, int /* min_i */, int /* min_j */,
      int /*max_i*/,
      int /*max_j*/)
  {
    if (!enabled_)
    {
      return;
    }

    if (!kernel_initialized_)
    {
      kernel_ = cv::getStructuringElement(type_, cv::Size(size_ / master_grid.getResolution(), size_ / master_grid.getResolution()));
      kernel_initialized_ = true;
    }

    cv::Mat mat(master_grid.getSizeInCellsY(), master_grid.getSizeInCellsX(), CV_8UC1, master_grid.getCharMap());

    // TODO: Move to another plugin
    const int padding = 0.1 / master_grid.getResolution();
    mat(cv::Rect(0, 0, padding, mat.rows)) = cv::Scalar(0);
    mat(cv::Rect(mat.cols - padding, 0, padding, mat.rows)) = cv::Scalar(0);
    mat(cv::Rect(0, 0, mat.cols, padding)) = cv::Scalar(0);
    mat(cv::Rect(0, mat.rows - padding, mat.cols, padding)) = cv::Scalar(0);

    cv::dilate(mat, mat, kernel_);
  }

} // namespace mep3_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mep3_navigation::DilationLayer, nav2_costmap_2d::Layer)
