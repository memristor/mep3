#include "mep3_navigation/opponent_tracking_layer/opponent_tracking_layer.hpp"

#define DEFAULT_COSTMAP_THRESHOLD 95
#define DEFAULT_OCCUPIED_VALUE 100
#define DEFAULT_COSTMAP_QOS rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()
#define ROS_NODE_NAME "opponent_tracking_layer"

namespace mep3_navigation
{

    OpponentTrackingLayer::OpponentTrackingLayer() : Node(ROS_NODE_NAME)
    {
        this->declare_parameter("costmap_threshold", rclcpp::ParameterValue(DEFAULT_COSTMAP_THRESHOLD));
        this->get_parameter(std::string {ROS_NODE_NAME} + "." + "costmap_threshold", this->costmap_threshold_);

        this->declare_parameter("occupied_value", rclcpp::ParameterValue(DEFAULT_OCCUPIED_VALUE));
        this->get_parameter(std::string {ROS_NODE_NAME} + "." + "occupied_value", this->occupied_value_);

        this->costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "global_costmap/costmap",
            DEFAULT_COSTMAP_QOS,
            std::bind(&OpponentTrackingLayer::costmap_callback, this, std::placeholders::_1));

        this->opponent_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "opponent_tracking/occupancy",
            DEFAULT_COSTMAP_QOS);

        this->occupancy_initialized_ = false;
        this->occupancy_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

        RCLCPP_INFO(this->get_logger(), "Opponent tracking node started");
    }

    void OpponentTrackingLayer::set_occupancy_header(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (!this->occupancy_initialized_)
        {
            RCLCPP_INFO(this->get_logger(), "Initialize opponent occupancy grid");
            this->occupancy_->set__header(msg->header);
            this->occupancy_->set__info(msg->info);
            this->occupancy_->data.resize(msg->data.size());
            std::fill(
                this->occupancy_->data.begin(),
                this->occupancy_->data.end(),
                0);
            this->occupancy_initialized_ = true;
        }
        else
        {
            const std::size_t msg_size = msg->data.size(),
                              occupancy_size = this->occupancy_->data.size();
            if (msg_size != occupancy_size)
            {
                RCLCPP_ERROR(this->get_logger(), "Costmap layer and occupancy map size mismatch: %li != %li", msg_size, occupancy_size);
                return;
            }
            this->occupancy_->set__header(msg->header);
            this->occupancy_->set__info(msg->info);
        }
    }

    void OpponentTrackingLayer::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_ERROR(this->get_logger(), "OpponentTrackingLayer::costmap_callback");

        this->set_occupancy_header(msg);

        auto costmap_it = msg->data.begin();
        auto occupancy_it = this->occupancy_->data.begin();
        while (costmap_it != msg->data.end() && occupancy_it != this->occupancy_->data.end())
        {
            // Occupancy probabilities are in the range [0,100].
            // Unknown is -1.
            if (*costmap_it >= this->costmap_threshold_)
            {
                *occupancy_it = this->occupied_value_;
            }
            ++costmap_it;
            ++occupancy_it;
        }
        this->opponent_pub_->publish(*this->occupancy_);
    }

} // namespace mep3_navigation

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mep3_navigation::OpponentTrackingLayer>());
    rclcpp::shutdown();
    return 0;
}