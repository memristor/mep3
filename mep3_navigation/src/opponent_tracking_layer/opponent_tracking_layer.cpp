#include "mep3_navigation/opponent_tracking_layer/opponent_tracking_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEFAULT_COSTMAP_QOS rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()

namespace mep3_navigation
{

    OpponentTrackingLayer::OpponentTrackingLayer() : Node("opponent_tracking_layer")
    {
        this->costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "local_costmap/costmap",
            DEFAULT_COSTMAP_QOS,
            std::bind(&OpponentTrackingLayer::costmap_callback, this, std::placeholders::_1));
        this->costmap_updates_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            "local_costmap/costmap_updates",
            DEFAULT_COSTMAP_QOS,
            std::bind(&OpponentTrackingLayer::costmap_updates_callback, this, std::placeholders::_1));

        this->opponent_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "opponent_tracking/occupancy",
            DEFAULT_COSTMAP_QOS);
        this->opponent_updates_pub_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
            "opponent_tracking/occupancy_updates",
            DEFAULT_COSTMAP_QOS);

        this->occupancy_initialized_ = false;
        this->occupancy_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

        RCLCPP_INFO(get_logger(), "Opponent tracking node started");
    }

    void OpponentTrackingLayer::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        this->opponent_pub_->publish(*msg);
    }

    void OpponentTrackingLayer::costmap_updates_callback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
    {
        this->opponent_updates_pub_->publish(*msg);
    }

} // namespace mep3_navigation

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mep3_navigation::OpponentTrackingLayer>());
    rclcpp::shutdown();
    return 0;
}