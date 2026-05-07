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

#ifndef MEP3_NAVIGATION__LIDAR_FILTER_HPP_
#define MEP3_NAVIGATION__LIDAR_FILTER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
//#include "geometry_msgs/msg/transform_stamped.hpp"
//#include "geometry_msgs/msg/twist.hpp"
#include <vector>

#define ZONE_COUNT 8
static double zones[ZONE_COUNT][4] =
{   
    {-1, -1.5, 1, -0.85}, // 1. zona
    {-1, -0.85, -0.2, -0.15}, // 2. zona
    {-0.2, -0.85, 1, -0.15}, // 3. zona
    {-1, -0.15, -0.2, 0.15}, // 4. zona
    {-0.2, -0.15, 1, 0.15}, // 5. zona
    {-1, 0.15, -0.2, 0.85}, // 6. zona
    {-0.2, 0.15, 1, 0.85}, // 7. zona
    {-1, 0.85, 1, 1.5} // 8. zona
};

namespace mep3_navigation{

    using msgType = sensor_msgs::msg::LaserScan;
    using oppRobot_pos = diagnostic_msgs::msg::KeyValue;

    class lidar_filter : public rclcpp::Node{
        public:
            lidar_filter(const rclcpp::NodeOptions &options);

        private:
            rclcpp::Subscription<msgType>::SharedPtr sub_;
            rclcpp::Publisher<msgType>::SharedPtr pub_;
            rclcpp::Publisher<oppRobot_pos>::SharedPtr robot_zone_pub_;
            
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            double robot_x, robot_y;
            double robot_rotation;

            bool debug_msg_;

            int opp_zone_index = -1;
            oppRobot_pos opp_zone_index_msg;
            double opp_x_sum = 0, opp_y_sum = 0;
            int lidar_scan_count = 0;


            bool is_opp_in_zone(int, double, double);
            void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            bool isInsideTable(float x, float y);
            void get_robot_pos();
    };
    
}

#endif