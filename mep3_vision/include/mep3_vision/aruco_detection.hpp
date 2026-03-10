#ifndef ARUCO_DETECTION_HPP
#define ARUCO_DETECTION_HPP

#include <functional>
#include <memory>

#include "mep3_msgs/action/aruco.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

typedef mep3_msgs::action::Aruco aruco_msg;

#define MARKER_ID_YELLOW 47
#define MARKER_ID_BLUE 36

#define CAMERA_FRONT_STR "front"
#define CAMERA_BACK_STR "back"
#define COLOR_BLUE_STR "blue"
#define COLOR_YELLOW_STR "yellow"

#define CAMERA_FRONT_SYMLINK "camera_front"
#define CAMERA_BACK_SYMLINK "camera_back"

#define CAMERA_FRONT_DEFAULT_INDEX 0
#define CAMERA_BACK_DEFAULT_INDEX 2

#define CAMERA_FRONT_WIDTH 1280
#define CAMERA_FRONT_HEIGHT 720

#define CAMERA_BACK_WIDTH 1280
#define CAMERA_BACK_HEIGHT 720

#define ARUCO_PICTURES_MAX 50

namespace mep3_vision
{
    class ArucoActionServer : public rclcpp::Node
    {
    public:

        using GoalHandleAruco = rclcpp_action::ServerGoalHandle<aruco_msg>;

        explicit ArucoActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        rclcpp_action::Server<aruco_msg>::SharedPtr action_server_;
        std::string camera_select;
        std::string color_;
        bool debug_;
        cv::VideoCapture videoFront, videoBack;
        uint8_t local_result_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const aruco_msg::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAruco> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandleAruco> goal_handle);

        void execute(const std::shared_ptr<GoalHandleAruco> goal_handle);

        void sortMarkers(std::vector<int> &markerIds, std::vector<std::vector<cv::Point2f>> &markerCorners);

        inline bool shouldFlipMarker(const int &markerId);
    
    };  // class ArucoActionServer

}  // namespace mep3_vision

#endif