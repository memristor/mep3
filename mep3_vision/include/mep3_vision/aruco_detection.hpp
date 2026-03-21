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

#define CAMERA_FRONT_SYMLINK "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2.7:1.0-video-index0"
#define CAMERA_BACK_SYMLINK "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2.6:1.0-video-index0"

#define CAMERA_FRONT_DEFAULT_INDEX 0
#define CAMERA_BACK_DEFAULT_INDEX 2

#define CAMERA_FRONT_WIDTH 1280
#define CAMERA_FRONT_HEIGHT 720

#define CAMERA_BACK_WIDTH 1280
#define CAMERA_BACK_HEIGHT 720

#define ARUCO_PICTURES_MAX 1
#define ARUCO_REGION_COUNT 4

static cv::Rect markerRegions[ARUCO_REGION_COUNT] =
{
    cv::Rect(100,   0, 320, 720),
    cv::Rect(420, 0, 220, 720),
    cv::Rect(640, 0, 220, 720),
    cv::Rect(860, 0, 320, 720)
};

namespace mep3_vision
{
    class ArucoActionServer : public rclcpp::Node
    {
    public:

        using GoalHandleAruco = rclcpp_action::ServerGoalHandle<aruco_msg>;

        explicit ArucoActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~ArucoActionServer();

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

        inline bool shouldFlipMarker(const int &markerId);

        bool markerInRegion(const std::vector<cv::Point2f>& corners, const cv::Rect& region);

        cv::Point2f getMarkerCenter(const std::vector<cv::Point2f>& corners);

        bool tryOpenFrontCamera(void);
        bool tryOpenBackCamera(void);
    
    };  // class ArucoActionServer

}  // namespace mep3_vision

#endif