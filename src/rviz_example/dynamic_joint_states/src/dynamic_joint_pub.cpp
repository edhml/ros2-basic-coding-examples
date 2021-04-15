#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class DynamicJointState: public rclcpp::Node {
public:
  DynamicJointState(): Node("dynamic_joint_state") {
    puber_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
    joint_msgs_ = std::make_shared<sensor_msgs::msg::JointState>();
    joint_msgs_->name.push_back("dynamic_joint_conti");
    joint_msgs_->position.push_back(0.0);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&DynamicJointState::timer_callback, this));
  }

private:
  void timer_callback() {
    counter_ += 0.3;
    joint_value_ = counter_;
    
    for(size_t i = 0; i < joint_msgs_->name.size(); ++i) {
      joint_msgs_->position[i] = joint_value_;
    }

    joint_msgs_->header.stamp = rclcpp::Clock().now();
    puber_->publish(*joint_msgs_);
  }

  double counter_ = 0.0;
  double joint_value_ = 0.0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr puber_;
  std::shared_ptr<sensor_msgs::msg::JointState> joint_msgs_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicJointState>());
  rclcpp::shutdown();
  return 0;
}
