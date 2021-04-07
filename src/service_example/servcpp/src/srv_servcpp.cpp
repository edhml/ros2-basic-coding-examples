#include "rclcpp/rclcpp.hpp"
#include "serv_msgs/srv/do_math.hpp"
#include <memory>

#define NO_ERROR 0
#define SYMBOL_ERR -1

void calc(const std::shared_ptr<serv_msgs::srv::DoMath::Request> request,
		std::shared_ptr<serv_msgs::srv::DoMath::Response> response) {
  long int num1, num2, rsl;
  rsl = 0;
  num1 = request->num1;
  num2 = request->num2;

  response->err = NO_ERROR;
  if(request->symbol == "+")
    rsl = num1 + num2;
  else if(request->symbol == "-")
    rsl = num1 - num2;
  else if(request->symbol == "*")
    rsl = num1 * num2;
  else if(request->symbol == "/")
    rsl = num1 / num2;
  else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Unknown symbol. Unable to complete request.");
    response->err = SYMBOL_ERR;
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The result is [%ld]", rsl);
}


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("do_math_server");

  rclcpp::Service<serv_msgs::srv::DoMath>::SharedPtr service =
	  node->create_service<serv_msgs::srv::DoMath>("/do_math_server/calc", &calc);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Do Math Server Ready...");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
