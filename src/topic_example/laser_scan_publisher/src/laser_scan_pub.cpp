#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define DEG2RAD M_PI / 180.0

using namespace std::chrono_literals;

class LaserScanPublisherSim: public rclcpp::Node {
public:
  LaserScanPublisherSim(): Node("laser_scan_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    timer_ = this->create_wall_timer(
      30ms, 
      std::bind(&LaserScanPublisherSim::timer_callback, this));

    laser_msgs_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    laser_msgs_->header.frame_id = "laser_frame_link";

    double angle_resolution = 2500;
    double start_angle = -450000;
    double stop_angle = 2250000;
    double scan_frequency = 2500;

    double angle_range = stop_angle - start_angle;
    double num_values = angle_range / angle_resolution;
    if (static_cast<int>(angle_range) % 
	static_cast<int>(angle_resolution) == 0)
      ++num_values;

    laser_msgs_->ranges.resize(static_cast<int>(num_values));

    laser_msgs_->time_increment =
      static_cast<float>((angle_resolution / 10000.0) / 360.0 / (scan_frequency / 100.0));
    laser_msgs_->angle_increment = static_cast<float>(angle_resolution / 10000.0 * DEG2RAD);
    laser_msgs_->angle_min = static_cast<float>(start_angle / 10000.0 * DEG2RAD - M_PI / 2);
    laser_msgs_->angle_max = static_cast<float>(stop_angle / 10000.0 * DEG2RAD - M_PI / 2);
    laser_msgs_->scan_time = static_cast<float>(100.0 / scan_frequency);
    laser_msgs_->range_min = 0.0f;
    laser_msgs_->range_max = 10.0f;

    counter_ = 0.0;
    amplitude_ = 1;
    distance_ = 0.0f;
    loop_ = 0;
  }

private:
  void timer_callback() {
    counter_ += 0.1;
    loop_ += 1;
    distance_ = static_cast<float>(std::abs(amplitude_));

    for (size_t i = 0; i < laser_msgs_->ranges.size(); ++i)
      laser_msgs_->ranges[i] = distance_;

    int dot = 1000 / 30; // 1s per circle
    int step = loop_ % dot;
    int start = step * dot;
    int end = (step + 1) * dot < (int)laser_msgs_->ranges.size() ? 
      (step + 1) * dot : (int)laser_msgs_->ranges.size();
    for (int i = start; i < end; ++i)
      laser_msgs_->ranges[i] = 0;

    laser_msgs_->header.stamp = rclcpp::Clock().now();
    publisher_->publish(*laser_msgs_); 
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_msgs_;

  double counter_;
  int amplitude_, loop_;
  float distance_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanPublisherSim>());
  rclcpp::shutdown();
  return 0;
}
