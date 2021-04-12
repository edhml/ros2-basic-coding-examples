#include <functional>
#include <memory>
#include <thread>

#include <algo_msgs/action/sort.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_example_cpp {

class BubbleSortActionServer: public rclcpp::Node {
public:
  BubbleSortActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("bubble_sort_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<algo_msgs::action::Sort>(
      this,
      "/merge_sort",
      std::bind(&BubbleSortActionServer::operate_goal, this, _1, _2),
      std::bind(&BubbleSortActionServer::operate_cancel, this, _1),
      std::bind(&BubbleSortActionServer::operate_accepted, this, _1));
  }

private:
  rclcpp_action::Server<algo_msgs::action::Sort>::SharedPtr action_server_;

  rclcpp_action::GoalResponse operate_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const algo_msgs::action::Sort::Goal> goal) {

    RCLCPP_INFO(this->get_logger(), "Received request");
    (void)goal;
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse operate_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<algo_msgs::action::Sort>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel operation.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void operate_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<algo_msgs::action::Sort>> goal_handle) {

    std::thread{
      std::bind(&BubbleSortActionServer::execute, this, std::placeholders::_1), 
      goal_handle
    }.detach();
  }

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<algo_msgs::action::Sort>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Execute goal");
    rclcpp::Rate rate(2);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<algo_msgs::action::Sort::Feedback>();
    auto& sequence = feedback->partial_sequence.data;
    sequence.clear();
    sequence = goal->random_sequence.data;
    auto result = std::make_shared<algo_msgs::action::Sort::Result>();

    int i, j;
    for(i = 0; (i < (int)sequence.size()-1) && rclcpp::ok(); i++) {    
      if(goal_handle->is_canceling()) {
        result->sorted_sequence.data = sequence;
	goal_handle->canceled(result);
	RCLCPP_INFO(this->get_logger(), "Goal canceled");
	return;
      }
      for(j = 0; j < (int)sequence.size()-i-1; j++) {
        if(sequence[j] > sequence[j+1]) {
	  swap(&sequence[j], &sequence[j+1]);
	  goal_handle->publish_feedback(feedback);
	  RCLCPP_INFO(this->get_logger(), "Publish feedback");
	  rate.sleep();
        }
      }
    }

    if(rclcpp::ok()) {
      result->sorted_sequence.data = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void swap(int *xp, int *yp) {
    int temp = *xp;
    *xp = *yp;
    *yp = temp;
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(action_example_cpp::BubbleSortActionServer)
