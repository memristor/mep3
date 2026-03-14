#include "mep3_vision/aruco_detection.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <bitset>

namespace mep3_vision
{
  using GoalHandleAruco = rclcpp_action::ServerGoalHandle<aruco_msg>;
   ArucoActionServer::ArucoActionServer(const rclcpp::NodeOptions & options)
  : Node("aruco_action_server", options)
  {
    using namespace std::placeholders;

    this->declare_parameter<std::string>("color", "blue");
    color_ = this->get_parameter("color").as_string();

    this->declare_parameter<bool>("debug", false);
    debug_ = this->get_parameter("debug").as_bool();

    videoFront.open(CAMERA_FRONT_SYMLINK, cv::CAP_V4L2);

    if (!videoFront.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start front camera via symlink");
        // try the default index instead
        videoFront.open(CAMERA_FRONT_DEFAULT_INDEX, cv::CAP_V4L2);
    }

    if (!videoFront.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start front camera");
    }
    else
    {
      // Set the resolution
      videoFront.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_FRONT_WIDTH);
      videoFront.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_FRONT_HEIGHT);

      // Set the MJPG format
      videoFront.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    }

    videoBack.open(CAMERA_BACK_SYMLINK, cv::CAP_V4L2);

    if (!videoBack.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start back camera via symlink");
        // try the default index instead
        videoBack.open(CAMERA_BACK_DEFAULT_INDEX, cv::CAP_V4L2);
    }

    if (!videoBack.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start back camera");
    }
    else
    {
      // Set the resolution
      videoBack.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_BACK_WIDTH);
      videoBack.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_BACK_HEIGHT);

      // Set the MJPG format
      videoBack.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    }

    this->action_server_ = rclcpp_action::create_server<aruco_msg>(this, "aruco", 
      std::bind(&ArucoActionServer::handle_goal, this, _1, _2),
      std::bind(&ArucoActionServer::handle_cancel, this, _1),
      std::bind(&ArucoActionServer::handle_accepted, this, _1)
    );
  }

  rclcpp_action::GoalResponse ArucoActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const aruco_msg::Goal> goal)
  {
    (void)uuid;
    if (goal->camera_select != CAMERA_FRONT_STR && goal->camera_select != CAMERA_BACK_STR)
      return rclcpp_action::GoalResponse::REJECT;

    camera_select = goal->camera_select;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse ArucoActionServer::handle_cancel(const std::shared_ptr<GoalHandleAruco> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void ArucoActionServer::handle_accepted(const std::shared_ptr<GoalHandleAruco> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&ArucoActionServer::execute, this, _1), goal_handle}.detach();
  }

  void ArucoActionServer::execute(const std::shared_ptr<GoalHandleAruco> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<mep3_msgs::action::Aruco::Result>();
    local_result_ = 0;

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;

    cv::Mat inputImage, inputImageGray;

    cv::VideoCapture &inputVideo = (camera_select == CAMERA_FRONT_STR) ? videoFront : videoBack;

    if(debug_)
      RCLCPP_INFO(this->get_logger(), "Camera selected: %s", camera_select.c_str());

    std::vector<int> markerIdsFiltered;
    std::vector<std::vector<cv::Point2f>> markerCornersFiltered;

    bool flipRegion[ARUCO_REGION_COUNT] = {false};
    int regionsToFlip = 0;
    for (int i = 0; i < ARUCO_PICTURES_MAX; ++i)
    {
      if (regionsToFlip == ARUCO_REGION_COUNT)
        break;

      if(!inputVideo.grab()){
        RCLCPP_INFO(this->get_logger(), "Grab failed");
        goal_handle->abort(result);
      }

      if(!inputVideo.retrieve(inputImage)){
        RCLCPP_INFO(this->get_logger(), "Retrieve failed");
        goal_handle->abort(result);
      }

      detector.detectMarkers(inputImage, markerCorners, markerIds);

      if(debug_){
        if (markerIds.size() > 0)
          cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds);

        for (int i = 0; i < ARUCO_REGION_COUNT; ++i)
          cv::rectangle(inputImage, markerRegions[i], cv::Scalar(0, 255, 0), 2);
      }

      for (size_t i = 0; i < markerIds.size(); ++i)
      {
        if (regionsToFlip == ARUCO_REGION_COUNT)
          break;
          
        cv::Point2f center = getMarkerCenter(markerCorners[i]);
        if (debug_)
          cv::circle(inputImage, center, 5, cv::Scalar(0,0,255), -1);

        for (int k = 0; k < ARUCO_REGION_COUNT; ++k)
        {
          if (flipRegion[k])
            continue;

          if (shouldFlipMarker(markerIds[i]) && markerInRegion(markerCorners[i], markerRegions[k]))
          {
            int mask = 1 << (ARUCO_REGION_COUNT - k - 1);
            local_result_ |= mask;
            flipRegion[k] = true;
            ++regionsToFlip;
            break;
          }
        }
      }

      if (debug_)
      {
        cv::imshow("Window", inputImage);
        cv::waitKey(1);
      }
    }

    result->result_mask = local_result_;
    RCLCPP_INFO( this->get_logger(), "Boards to flip: %s", 
      std::bitset<sizeof(int) * CHAR_BIT>{static_cast<unsigned int>(local_result_)}.to_string().c_str());

    goal_handle->succeed(result);
  }

  cv::Point2f ArucoActionServer::getMarkerCenter(const std::vector<cv::Point2f>& corners)
  {
      cv::Point2f center(0, 0);

      for (const auto& p : corners)
          center += p;

      center *= (1.0f / corners.size());
      return center;
  }

  bool ArucoActionServer::markerInRegion(const std::vector<cv::Point2f>& corners, const cv::Rect& region)
  {
      cv::Point2f center = getMarkerCenter(corners);
      return region.contains(center);
  }

  inline bool ArucoActionServer::shouldFlipMarker(const int &markerId)
  {
    return ((color_ == COLOR_BLUE_STR && markerId == MARKER_ID_YELLOW) ||
    (color_ == COLOR_YELLOW_STR && markerId == MARKER_ID_BLUE));
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(mep3_vision::ArucoActionServer)