#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#define ARUCO_TAG_LENGTH_M 0.05
#define AXIS_LENGTH_M 0.1

int main() {
  std::string image_path = cv::samples::findFile("img/top_yellow.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  if (img.empty()) {
    std::cout << "Could not read the image: " << image_path << std::endl;
    return 1;
  }
  cv::imshow("Display window", img);

  // GENERATION
  // cv::Mat markerImage;
  // cv::Ptr<cv::aruco::Dictionary> dictionary =
  // cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  // cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
  // cv::imwrite("marker23.png", markerImage);

  // DETECTION
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters =
      cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
  cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds,
                           parameters, rejectedCandidates);

  cv::Mat outputImage = img.clone();
  if (markerIds.size() > 0)
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

  cv::imshow("output", outputImage);

  cv::Mat cameraMatrix, distCoeffs;
  // To work with examples from the tutorial, you can use these camera
  // parameters:
  cv::Size imgSize = img.size();
  cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
  cameraMatrix.at<double>(0, 0) = cameraMatrix.at<double>(1, 1) = 650;
  cameraMatrix.at<double>(0, 2) = imgSize.width / 2;
  cameraMatrix.at<double>(1, 2) = imgSize.height / 2;
  distCoeffs = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));

  // POSE ESTIMATION
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(markerCorners, ARUCO_TAG_LENGTH_M,
                                       cameraMatrix, distCoeffs, rvecs, tvecs);

  cv::Mat outputImage2 = img.clone();
  for (int i = 0; i < rvecs.size(); ++i) {
    auto rvec = rvecs[i];
    auto tvec = tvecs[i];
    cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvec, tvec,
                        AXIS_LENGTH_M);
  }

  cv::imshow("output2", outputImage);
  int k = 0;
  while (k != 27) k = cv::waitKey(0);  // Wait for a keystroke in the window

  return 0;
}
