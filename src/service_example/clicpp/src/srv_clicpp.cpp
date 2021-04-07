#include "rclcpp/rclcpp.hpp"
#include "serv_msgs/srv/do_math.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  if (argc != 4) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: ros2 run clicpp srv_clicpp 50 101 '*'");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("do_math_client");
  rclcpp::Client<serv_msgs::srv::DoMath>::SharedPtr client = 
    node->create_client<serv_msgs::srv::DoMath>("/do_math_server/calc");

  auto request = std::make_shared<serv_msgs::srv::DoMath::Request>();
  request->num1 = atoll(argv[1]);
  request->num2 = atoll(argv[2]);
  request->symbol = argv[3];

  while(!client->wait_for_service(1s)) {
    if(!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not avaiable, waiting...");
  }

  auto result = client->async_send_request(request);

  if(rclcpp::spin_until_future_complete(node, result) ==
     rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success to request service");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The service response: [%d]", result.get()->err);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to request service");
  }

  rclcpp::shutdown();
  return 0;
}
