#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

namespace mep3_driver
{
class LaserInflator : public rclcpp::Node
{
public:
  LaserInflator() : Node("laser_inflator")
  {
    this->declare_parameter("inflation_radius", 0.1);
    this->get_parameter("inflation_radius", inflation_radius_);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/big/scan", 10, std::bind(&LaserInflator::process_scan, this, _1));
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/big/scan_inflated", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  float inflation_radius_;

  void process_scan(const sensor_msgs::msg::LaserScan & msg) const
  {
    sensor_msgs::msg::LaserScan scan_out;
    scan_out.header = msg.header;
    scan_out.angle_min = -M_PI;
    scan_out.angle_max = M_PI;
    scan_out.angle_increment = M_PI / 180.0;
    scan_out.range_max = std::numeric_limits<float>::infinity();

    size_t ranges_size = static_cast<size_t>(
      std::ceil((scan_out.angle_max - scan_out.angle_min) / scan_out.angle_increment));
    scan_out.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

    float point_angle = msg.angle_min;  // angle of current point from incoming laser data
    // iterate through received laser points
    for (auto it = msg.ranges.begin(); it != msg.ranges.end(); it++) {
      // create new points around current laser point
      // new points lay on a semicircle facing the laser frame
      float curr_angle = point_angle - 3.0 * M_PI_2;
      const float end_angle = point_angle - M_PI_2;

      // TODO(angstrem98): ovaj step treba otpimalno odabrati kako se ne bi trosio previse CPU
      // ili naci mozda neki drugi nacin
      const float step = M_PI / 45.0;
      for (; curr_angle <= end_angle; curr_angle += step) {
        const float point_range = *it;
        const float new_x = point_range * cosf(point_angle) + inflation_radius_ * cosf(curr_angle);
        const float new_y = point_range * sinf(point_angle) + inflation_radius_ * sinf(curr_angle);
        const float new_range = std::hypot(new_x, new_y);
        const float new_angle = atan2f(new_y, new_x);

        const int index =
          static_cast<int>((new_angle - scan_out.angle_min) / scan_out.angle_increment);
        if (new_range < scan_out.ranges.at(index)) {
          scan_out.ranges.at(index) = new_range;
        }
      }

      // prepare for next incoming point
      point_angle += msg.angle_increment;
    }

    publisher_->publish(scan_out);
  }
};

}  // namespace mep3_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mep3_driver::LaserInflator>());
  rclcpp::shutdown();

  return 0;
}