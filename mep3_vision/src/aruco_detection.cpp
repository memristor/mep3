#include <functional>
#include <memory>
#include <thread>

#include "mep3_msgs/action/aruco.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cassert>
#include <algorithm>

typedef mep3_msgs::action::Aruco aruco_msg;

#define MARKER_ID_YELLOW 1
#define MARKER_ID_BLUE 2

#define CAMERA_FRONT_STR "front"
#define CAMERA_BACK_STR "back"
#define COLOR_BLUE_STR "blue"
#define COLOR_YELLOW_STR "yellow"

#define CAMERA_FRONT_SYMLINK "camera_front"
#define CAMERA_BACK_SYMLINK "camera_back"

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
        RCLCPP_INFO(this->get_logger(), "Failed to start front camera via symlink");
        // try the index 0 instead
        videoFront.open(0);
    }

    if (!videoFront.isOpened())
    {
      RCLCPP_INFO(this->get_logger(), "Failed to start front camera");
      return;
    }

    videoBack.open(CAMERA_BACK_SYMLINK);

    if (!videoBack.isOpened())
    {
        RCLCPP_INFO(this->get_logger(), "Failed to start back camera via symlink");
        // try the index 1 instead
        videoFront.open(1);
    }

    if (!videoBack.isOpened())
    {
      RCLCPP_INFO(this->get_logger(), "Failed to start back camera");
      return;
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

 rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const aruco_msg::Goal> goal){
  (void)uuid;
  if (goal->color != COLOR_BLUE_STR && goal->color != COLOR_YELLOW_STR)
    return rclcpp_action::GoalResponse::REJECT;

  camera_select = goal->camera_select;
  color = goal->color;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
 }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAruco> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel gas");
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
    result->result_mask = 0;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams =
        cv::aruco::DetectorParameters::create();

    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;

    cv::Mat inputImage, inputImageGray;

    cv::VideoCapture inputVideo = (camera_select == CAMERA_FRONT_STR) ? videoFront : videoBack;
    inputVideo.retrieve(inputImage);
    
    cv::cvtColor(inputImage, inputImageGray, cv::COLOR_BGR2GRAY);

    cv::aruco::detectMarkers(
        inputImageGray,
        dictionary,
        markerCorners,
        markerIds,
        detectorParams
    );

    sortMarkers(markerIds, markerCorners);
    for (size_t i = 0; i < markerIds.size(); ++i)
    {
      if (shouldFlipMarker(i))
      {
        int mask = 1 << i;
        result->result_mask |= mask;
      }
    }
  }

  void sortMarkers(std::vector<int> &markerIds, std::vector<std::vector<cv::Point2f>> &markerCorners)
  {
    assert(markerIds.size() == markerCorners.size() && "markerIds size is not equal to markerCorners size!");
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
      markerPairs[i].markerId = markerIds[i];
      markerPairs[i].markerCorner = markerCorners[i];
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