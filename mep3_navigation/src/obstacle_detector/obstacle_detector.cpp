// Copyright 2022 Memristor Robotics
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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;

class ObstacleDetector : public rclcpp::Node {
 public:
  ObstacleDetector() : Node("obstacle_detector") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan_inflated", 10,
            std::bind(&ObstacleDetector::laser_scan_callback, this, _1));
    range_fl_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(
        "binary_ranger_command/front_left", 10,
        std::bind(&ObstacleDetector::range_callback_fl, this, _1));
    range_fr_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(
        "binary_ranger_command/front_right", 10,
        std::bind(&ObstacleDetector::range_callback_fr, this, _1));
    range_rl_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(
        "binary_ranger_command/rear_left", 10,
        std::bind(&ObstacleDetector::range_callback_rl, this, _1));
    range_rr_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(
        "binary_ranger_command/rear_right", 10,
        std::bind(&ObstacleDetector::range_callback_rr, this, _1));
    publisher_ =
        this->create_publisher<std_msgs::msg::Bool>("obstacle_detected", 10);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_fl_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_fr_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_rl_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_rr_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  bool detection_fl = false;
  bool detection_fr = false;
  bool detection_rl = false;
  bool detection_rr = false;

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    sensor_msgs::msg::LaserScan scan = *msg;
  }

  void range_callback_fl(const sensor_msgs::msg::Range::SharedPtr msg) {
    sensor_msgs::msg::Range scan = *msg;
    detection_fl = scan.range > 0;
    publish();
  }

  void range_callback_fr(const sensor_msgs::msg::Range::SharedPtr msg) {
    sensor_msgs::msg::Range scan = *msg;
    detection_fr = scan.range > 0;
    publish();
  }

  void range_callback_rl(const sensor_msgs::msg::Range::SharedPtr msg) {
    sensor_msgs::msg::Range scan = *msg;
    detection_rl = scan.range > 0;
    publish();
  }

  void range_callback_rr(const sensor_msgs::msg::Range::SharedPtr msg) {
    sensor_msgs::msg::Range scan = *msg;
    detection_rr = scan.range > 0;
    publish();
  }

  void publish(){
    publisher_->publish(detection_fl | detection_fr | detection_rl | detection_rr);
  }
};


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();

  return 0;
}
