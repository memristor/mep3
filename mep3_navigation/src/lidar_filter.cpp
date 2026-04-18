
#include "mep3_navigation/lidar_filter.hpp"
#include <cmath>

namespace mep3_navigation{

    using msgType = sensor_msgs::msg::LaserScan;
    //using transform =  geometry_msgs::msg::TransformStamped;

    lidar_filter::lidar_filter() : Node("Lidar_filter"){
        sub_ = this->create_subscription<msgType>("scan", 10, [this](const msgType::SharedPtr msg){this->callback(msg);});
        pub_ = this->create_publisher<msgType>("FilteredScan", 10);

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
                }
            }
            angle += msg->angle_increment;
        }

        pub_->publish(filtered);
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
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
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

