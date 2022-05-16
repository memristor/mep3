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

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

namespace mep3_driver
{
class LaserInflator : public rclcpp::Node
{
public:
  LaserInflator()
  : Node("laser_inflator")
  {
    this->declare_parameter("inflation_radius", 0.2);
    this->declare_parameter("inflation_angular_step", 0.09);
    this->get_parameter("inflation_radius", inflation_radius_);
    this->get_parameter("inflation_angular_step", inflation_angular_step_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(rclcpp::SensorDataQoS()),
      std::bind(&LaserInflator::scan_callback, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_inflated", rclcpp::QoS(rclcpp::SensorDataQoS()));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  float inflation_radius_;
  float inflation_angular_step_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  void inflate_scan(sensor_msgs::msg::LaserScan & scan)
  {
    sensor_msgs::msg::LaserScan scan_out;
    scan_out.header = scan.header;
    scan_out.angle_min = -M_PI;
    scan_out.angle_max = M_PI;
    scan_out.angle_increment = M_PI / 180.0;
    scan_out.range_max = std::numeric_limits<float>::infinity();

    size_t ranges_size = static_cast<size_t>(
      std::ceil((scan_out.angle_max - scan_out.angle_min) / scan_out.angle_increment));
    scan_out.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    float point_angle = scan.angle_min;  // angle of current point from incoming laser data
    // iterate through received laser points
    for (auto it = scan.ranges.begin(); it != scan.ranges.end();
      it++, point_angle += scan.angle_increment)
    {
      if (*it == std::numeric_limits<float>::infinity()) {
        continue;
      }

      // create new points around current laser point
      // new points lay on a semicircle facing the laser frame
      float curr_angle = point_angle - 3.0 * M_PI_2;
      const float end_angle = point_angle - M_PI_2;

      // TODO(angstrem98): ovaj step treba otpimalno odabrati kako se ne bi trosio previse CPU
      // ili naci mozda neki drugi nacin
      for (; curr_angle <= end_angle; curr_angle += inflation_angular_step_) {
        const float point_range = *it;
        const float new_x = point_range * cosf(point_angle) + inflation_radius_ * cosf(curr_angle);
        const float new_y = point_range * sinf(point_angle) + inflation_radius_ * sinf(curr_angle);
        const float new_range = std::hypot(new_x, new_y);
        const float new_angle = atan2f(new_y, new_x);

        int index =
          static_cast<int>((new_angle - scan_out.angle_min) / scan_out.angle_increment);
        // TODO(angstrem98) OVO PROVERI
        if (index >= 359) {
          index = 359;
        }
        if (new_range < scan_out.ranges.at(index)) {
          scan_out.ranges.at(index) = new_range;
        }
      }
    }

    scan = scan_out;
  }

  bool constrain_scan(sensor_msgs::msg::LaserScan & scan)
  {
    std::string to_frame = "map";
    std::string from_frame = "laser";
    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
      transform_stamped = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s", to_frame.c_str(),
        from_frame.c_str(), ex.what());
      return false;
    }

    float transform_angle = static_cast<float>(tf2::getYaw(transform_stamped.transform.rotation));
    const double x_offset = transform_stamped.transform.translation.x;
    const double y_offset = transform_stamped.transform.translation.y;

    // iterate through points and check if xy coords are valid

    float point_angle = scan.angle_min + transform_angle;  // apply rotation
    for (auto it = scan.ranges.begin(); it != scan.ranges.end();
      it++, point_angle += scan.angle_increment)
    {
      if (*it == std::numeric_limits<float>::infinity()) {
        continue;
      }
      const float point_range = *it;
      const double x = point_range * cosf(point_angle) + x_offset;
      const double y = point_range * sinf(point_angle) + y_offset;

      // Remove total stop button that out lidar can see
      if (point_range < 0.18) {
        *it = std::numeric_limits<float>::infinity();
        continue;
      }

      // Is (x, y) valid?
      // Currently, just check if the point is inside a rectangle
      // a bit smaller than the playing area.
      const double shrink = 0.1;
      if (
        (x >= -1.5 + shrink) && (x <= 1.5 - shrink) && (y >= -1.0 + shrink) &&
        (y <= 1.0 - shrink))
      {
        // point is valid, don't touch it, continue
        continue;
      } else {
        // point outside area, delete it by making the range equal to infinity
        *it = std::numeric_limits<float>::infinity();
      }
    }
    return true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan scan = *msg;
    if (constrain_scan(scan)) {
      inflate_scan(scan);
      publisher_->publish(scan);
    }
  }
};

}  // namespace mep3_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mep3_driver::LaserInflator>());
  rclcpp::shutdown();

  return 0;
}
