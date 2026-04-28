
#include "mep3_navigation/lidar_filter.hpp"
#include <cmath>
#include <string>

namespace mep3_navigation{

    using msgType = sensor_msgs::msg::LaserScan;
    //using oppRobot_pos = std_msgs::msg::UInt8;
    using oppRobot_pos = diagnostic_msgs::msg::KeyValue;

    lidar_filter::lidar_filter() : Node("Lidar_filter"){
        sub_ = this->create_subscription<msgType>("scan", 10, [this](const msgType::SharedPtr msg){this->callback(msg);});
        pub_ = this->create_publisher<msgType>("FilteredScan", 10);
        robot_zone_pub_ = this->create_publisher<oppRobot_pos>("shared_blackboard", rclcpp::SystemDefaultsQoS().reliable().transient_local());
        debug_msg_ = false;
        opp_zone_index_msg.key = "opp_robot_zone";
        opp_zone_index_msg.value = "0";

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void lidar_filter::callback(const msgType::SharedPtr msg) {
        auto filtered = *msg;
        float angle = msg->angle_min;

        get_robot_pos();

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float r = msg->ranges[i];

            if (std::isfinite(r)) {
                float x = r * cosf(angle + robot_rotation) + robot_x;
                float y = r * sinf(angle  + robot_rotation) + robot_y;

                if (!isInsideTable(x, y)) {
                    filtered.ranges[i] = std::numeric_limits<float>::infinity();
                } else {
                    for (int zone_index = 0; zone_index < ZONE_COUNT; zone_index++) {
                        if (is_opp_in_zone(zone_index, x, y)) {
                            if (opp_zone_index == zone_index) continue;

                            opp_zone_index = zone_index;
                            int temp = 0x01 << zone_index; 
                            opp_zone_index_msg.value = std::to_string(temp);
                            robot_zone_pub_->publish(opp_zone_index_msg);

                            break;
                        }
                    }
                }
            }
            angle += msg->angle_increment;
        }

        pub_->publish(filtered);
    }

    bool lidar_filter::is_opp_in_zone(int zone_index, double opp_x, double opp_y) {
        double x_min = zones[zone_index][0];
        double y_min = zones[zone_index][1];
        double x_max = zones[zone_index][2];
        double y_max = zones[zone_index][3];

        return (opp_x >= x_min && opp_x < x_max) && (opp_y >= y_min && opp_y < y_max);
    }

    bool lidar_filter::isInsideTable(float x, float y) {
        return (x >= -0.95 && x <= 0.95 && y >= -1.45 && y <= 1.45);
    }

    void lidar_filter::get_robot_pos(){
        geometry_msgs::msg::TransformStamped t;

        try {
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex) {
            if(!debug_msg_){
                RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
                debug_msg_ = true;
            }
    
            return;
        }

        robot_x = t.transform.translation.x;
        robot_y = t.transform.translation.y;

        tf2::Quaternion q;
        tf2::fromMsg(t.transform.rotation, q);
        double roll, pitch;

        tf2::Matrix3x3(q).getRPY(roll, pitch, robot_rotation);
    }
    
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mep3_navigation::lidar_filter>());
    rclcpp::shutdown();
    return 0;
}

