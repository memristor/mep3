#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
// using namespace cv;
int main() {
  std::string image_path = cv::samples::findFile("img/top_yellow.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  if (img.empty()) {
    std::cout << "Could not read the image: " << image_path << std::endl;
    return 1;
  }
  cv::imshow("Display window", img);
  int k = cv::waitKey(0);  // Wait for a keystroke in the window
  if (k == 's') {
    cv::imwrite("starry_night.png", img);
  }
  return 0;
}
