#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

class LaserScanSubscriberSim: public rclcpp::Node {
public:
  LaserScanSubscriberSim(): Node("laser_scan_subscriber") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LaserScanSubscriberSim::scan_callback, this, _1));
    count_ = 0;
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if(count_ % 33 == 0)
      RCLCPP_INFO(this->get_logger(), "Laser scan heartbeak alive...");
    count_++;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_msgs_;

  int count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanSubscriberSim>());
  rclcpp::shutdown();
  return 0;
}
