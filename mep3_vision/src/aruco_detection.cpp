#include <functional>
#include <memory>

#include "mep3_msgs/action/aruco.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <algorithm>

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

namespace mep3_vision
{
  
class ArucoActionServer : public rclcpp::Node
{
public:

  using GoalHandleAruco = rclcpp_action::ServerGoalHandle<aruco_msg>;

  explicit ArucoActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("aruco_action_server", options)
  {
    using namespace std::placeholders;

    videoFront.open(CAMERA_FRONT_SYMLINK);

    if (!videoFront.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start front camera via symlink");
        // try the default index instead
        videoFront.open(CAMERA_FRONT_DEFAULT_INDEX);
    }

    if (!videoFront.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start front camera");
      
    }

    videoBack.open(CAMERA_BACK_SYMLINK);

    if (!videoBack.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start back camera via symlink");
        // try the default index instead
        videoBack.open(CAMERA_BACK_DEFAULT_INDEX);
    }

    if (!videoBack.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start back camera");

    }

    this->action_server_ = rclcpp_action::create_server<aruco_msg>(this, "aruco", 
      std::bind(&ArucoActionServer::handle_goal, this, _1, _2),
      std::bind(&ArucoActionServer::handle_cancel, this, _1),
      std::bind(&ArucoActionServer::handle_accepted, this, _1)
    );
  }

private:
  rclcpp_action::Server<aruco_msg>::SharedPtr action_server_;
  std::string camera_select, color;
  cv::VideoCapture videoFront, videoBack;
  uint8_t local_result_;

 rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const aruco_msg::Goal> goal){
  (void)uuid;
  if (goal->color != COLOR_BLUE_STR && goal->color != COLOR_YELLOW_STR)
    return rclcpp_action::GoalResponse::REJECT;

  camera_select = goal->camera_select;
  color = goal->color;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
 }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAruco> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleAruco> goal_handle){
    using namespace std::placeholders;
    std::thread{std::bind(&ArucoActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleAruco> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<mep3_msgs::action::Aruco::Result>();
    local_result_ = 0;

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;

    cv::Mat inputImage, inputImageGray;

    cv::VideoCapture &inputVideo = (camera_select == CAMERA_FRONT_STR) ? videoFront : videoBack;

    RCLCPP_INFO(this->get_logger(), "Camera selected: %s", camera_select.c_str());

    if(!inputVideo.grab()){
      RCLCPP_INFO(this->get_logger(), "Grab failed");
      goal_handle->abort(result);
    }

    if(!inputVideo.retrieve(inputImage)){
      RCLCPP_INFO(this->get_logger(), "Retrive failed");
      goal_handle->abort(result);
    }

    detector.detectMarkers(inputImage, markerCorners, markerIds);

    sortMarkers(markerIds, markerCorners);
    for (size_t i = 0; i < markerIds.size(); ++i)
    {
      if (shouldFlipMarker(markerIds[i]))
      {
        int mask = 1 << (markerIds.size() - i - 1);
        local_result_ |= mask;
      }
    }

    result->result_mask = local_result_;
    RCLCPP_INFO(this->get_logger(), "Boards to flip: %x", local_result_);

    goal_handle->succeed(result);
  }

  void sortMarkers(std::vector<int> &markerIds, std::vector<std::vector<cv::Point2f>> &markerCorners)
  {
    // helper structs for sorting
    struct markerPair
    {
      int markerId;
      std::vector<cv::Point2f> markerCorner;
    };

    struct less_than_key
    {
        inline bool operator() (const struct markerPair pair1, const struct markerPair pair2)
        {
          float leftmostCorner1 = std::min(std::min(pair1.markerCorner[0].x, pair1.markerCorner[1].x),
          std::min(pair1.markerCorner[2].x, pair1.markerCorner[3].x));
          float leftmostCorner2 = std::min(std::min(pair2.markerCorner[0].x, pair2.markerCorner[1].x),
          std::min(pair2.markerCorner[2].x, pair2.markerCorner[3].x));
          return (leftmostCorner1 < leftmostCorner2);
        }
    };

    std::vector<struct markerPair> markerPairs;
    for (size_t i = 0; i < markerIds.size(); ++i)
    {
      markerPairs.push_back({markerIds[i], markerCorners[i]});
    }

    std::sort(markerPairs.begin(), markerPairs.end(), less_than_key());
    for (size_t i = 0; i < markerIds.size(); ++i)
    {
      markerIds[i] = markerPairs[i].markerId;
      markerCorners[i] = markerPairs[i].markerCorner;
    }
  }

  inline bool shouldFlipMarker(const int &markerId)
  {
    return ((color == COLOR_BLUE_STR && markerId == MARKER_ID_YELLOW) ||
    (color == COLOR_YELLOW_STR && markerId == MARKER_ID_BLUE));
  }
  
};  // class ArucoActionServer

}  // namespace mep3_vision

RCLCPP_COMPONENTS_REGISTER_NODE(mep3_vision::ArucoActionServer)