#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "algo_msgs/action/sort.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_example_cpp {

class BubbleSortActionClient: public rclcpp::Node {
public:
  BubbleSortActionClient(const rclcpp::NodeOptions& options)
  : Node("bubble_sort_action_client", options) {
    this->client_ptr_ = rclcpp_action::create_client<algo_msgs::action::Sort>(
      this,
      "/merge_sort");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&BubbleSortActionClient::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    if(!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not avaiable.");
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

    auto send_goal_options = rclcpp_action::Client<algo_msgs::action::Sort>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
      std::bind(&BubbleSortActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = 
      std::bind(&BubbleSortActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = 
      std::bind(&BubbleSortActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<algo_msgs::action::Sort>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(
      std::shared_future<rclcpp_action::ClientGoalHandle<algo_msgs::action::Sort>::SharedPtr> future) {
      auto goal_handle = future.get();
    if(!goal_handle)
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    else
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<algo_msgs::action::Sort>::SharedPtr,
      const std::shared_ptr<const algo_msgs::action::Sort::Feedback> feedback) {
    std::stringstream ss;
    for(auto number: feedback->partial_sequence.data)
      ss << number << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());  
  }

  void result_callback(
      const rclcpp_action::ClientGoalHandle<algo_msgs::action::Sort>::WrappedResult& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Client result callback");
    std::stringstream ss;
    ss << "Result received: ";
    for(auto number: result.result->sorted_sequence.data)
      ss << number << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(action_example_cpp::BubbleSortActionClient)
