// Copyright 2021 Memristor Robotics
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

#ifndef MEP3_BEHAVIOR_TREE__POSE_2D_HPP_
#define MEP3_BEHAVIOR_TREE__POSE_2D_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"

namespace BT
{
struct Pose2D
{
  double x, y, theta;
};


inline Pose2D mirrorPose(const Pose2D &pose)  {
  Pose2D p = pose;
  p.y *= -1;
  //p.theta = ((p.theta >= 0) ? 90.0 : -90.0) - p.theta;
  return p;
}

// Reference: https://www.behaviortree.dev/tutorial_03_generic_ports/
template<>
inline Pose2D convertFromString(StringView str)
{
  // We expect real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  } else {
    Pose2D output;
    output.x = convertFromString<double>(parts[0]);
    output.y = convertFromString<double>(parts[1]);
    output.theta = convertFromString<double>(parts[2]);
    return output;
  }
}

template<>
inline std::vector<Pose2D> convertFromString(StringView str)
{
  std::vector<Pose2D> output;

  auto posesParts = splitString(str, '|');
  for (auto &posePart : posesParts) {
    // We expect real numbers separated by semicolons
    auto parts = splitString(posePart, ';');
    if (parts.size() != 3) {
      throw RuntimeError("invalid input)");
    } else {
      Pose2D pose;
      pose.x = convertFromString<double>(parts[0]);
      pose.y = convertFromString<double>(parts[1]);
      pose.theta = convertFromString<double>(parts[2]);
      output.push_back(pose);
    }
  }

  return output;
}

template<>
inline std::vector<geometry_msgs::msg::Point> convertFromString(StringView str)
{
  std::vector<geometry_msgs::msg::Point> output;
  auto pointsParts = splitString(str, '|');
  for (auto &pointPart : pointsParts) {
    auto parts = splitString(pointPart, ';');
    if (parts.size() != 2 && parts.size() != 3) {
      throw RuntimeError("invalid input)");
    } else {
      geometry_msgs::msg::Point point;
      point.x = convertFromString<double>(parts[0]);
      point.y = convertFromString<double>(parts[1]);
      if (parts.size() == 3) {
        point.z = convertFromString<double>(parts[2]);
      }
      output.push_back(point);
    }
  }
  return output;
}

Pose2D& operator+=(Pose2D& lhs, const Pose2D& rhs)
{
  lhs.x += rhs.x;
  lhs.y += rhs.y;
  lhs.theta += rhs.theta;
  return lhs;
}

std::vector<Pose2D>& operator+=(std::vector<Pose2D>& lhs, const std::vector<Pose2D> rhs)
{
  assert(lhs.size() == rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    lhs[i] += rhs[i];
  }
  return lhs;
}

}  // namespace BT

#endif  // MEP3_BEHAVIOR_TREE__POSE_2D_HPP_
