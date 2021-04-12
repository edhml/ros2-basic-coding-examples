#include <memory>
#include <string>
#include <sstream>

#include "algo_msgs/action/sort.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node= rclcpp::Node::make_shared("bubble_sort_action_client");
  auto action_client = rclcpp_action::create_client<algo_msgs::action::Sort>(node, "/merge_sort");

  if(!action_client->wait_for_action_server(std::chrono::milliseconds(500))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not avaiable.");
    rclcpp::shutdown();
  }

  auto goal_msg = algo_msgs::action::Sort::Goal();
  goal_msg.random_sequence.data.clear();
  goal_msg.random_sequence.data.push_back(3);
  goal_msg.random_sequence.data.push_back(25);
  goal_msg.random_sequence.data.push_back(17);
  goal_msg.random_sequence.data.push_back(9);
  goal_msg.random_sequence.data.push_back(55);
  goal_msg.random_sequence.data.push_back(16);
  goal_msg.random_sequence.data.push_back(19);

  RCLCPP_INFO(node->get_logger(), "Sending goal");

  auto goal_handle_future = action_client->async_send_goal(goal_msg);
  if(rclcpp::spin_until_future_complete(node, goal_handle_future) !=
     rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Send goal call failed.");
    rclcpp::shutdown();
  }

  rclcpp_action::ClientGoalHandle<algo_msgs::action::Sort>::SharedPtr goal_handle = 
    goal_handle_future.get();
  if(!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
  }

  auto result_future = action_client->async_get_result(goal_handle);

  auto wait_result = rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(3));

  if(rclcpp::FutureReturnCode::TIMEOUT == wait_result) {
    RCLCPP_INFO(node->get_logger(), "canceling goal");
    auto cancel_result_future = action_client->async_cancel_goal(goal_handle);

    if (rclcpp::spin_until_future_complete(node, cancel_result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "failed to cancel goal");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(node->get_logger(), "goal is being canceled");
  } else if (rclcpp::FutureReturnCode::SUCCESS != wait_result) {
      RCLCPP_ERROR(node->get_logger(), "failed to get result");
      rclcpp::shutdown();
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "get result call failed.");
  }

  rclcpp_action::ClientGoalHandle<algo_msgs::action::Sort>::WrappedResult wrapped_result = result_future.get();
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      break;
  }

  rclcpp::shutdown();
  return 0;
}
