#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_interfaces/action/navigate.hpp>
#include <iostream>
#include <thread>

namespace nav_project
{
class UIClient : public rclcpp::Node
{
public:
  using Navigate = nav_interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

  explicit UIClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ui_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Navigate>(this, "navigate_robot");

    // Launch the terminal UI in a completely separate thread so it doesn't block ROS callbacks
    ui_thread_ = std::thread(&UIClient::ui_loop, this);
  }

  ~UIClient()
  {
    if (ui_thread_.joinable()) {
      ui_thread_.join();
    }
  }

private:
  rclcpp_action::Client<Navigate>::SharedPtr client_ptr_;
  std::thread ui_thread_;
  bool is_navigating_ = false;
  std::shared_ptr<GoalHandleNavigate> active_goal_handle_;

  // --- THE TERMINAL UI LOOP ---
  void ui_loop()
  {
    while (rclcpp::ok()) {
      if (!is_navigating_) {
        std::cout << "\n=== Robot Navigation Menu ===\n";
        std::cout << "Enter target X coordinate: ";
        double x; std::cin >> x;
        std::cout << "Enter target Y coordinate: ";
        double y; std::cin >> y;
        std::cout << "Enter target Theta (radians): ";
        double theta; std::cin >> theta;

        send_goal(x, y, theta);
      } else {
        std::cout << "\n[Robot is moving] Type 'c' and press Enter to CANCEL: ";
        char command;
        std::cin >> command;
        if (command == 'c' || command == 'C') {
          cancel_goal();
        }
      }
    }
  }

  // --- ACTION CLIENT METHODS ---
  void send_goal(double x, double y, double theta)
  {
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = Navigate::Goal();
    goal_msg.target_x = x;
    goal_msg.target_y = y;
    goal_msg.target_theta = theta;

    RCLCPP_INFO(this->get_logger(), "Sending goal...");

    auto send_goal_options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    
    // Bind the callbacks for response, feedback, and result
    send_goal_options.goal_response_callback =
      std::bind(&UIClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&UIClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&UIClient::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    is_navigating_ = true;
  }

  void cancel_goal()
  {
    if (active_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Sending cancel request...");
      this->client_ptr_->async_cancel_goal(active_goal_handle_);
    }
  }

  // --- CALLBACKS ---
  void goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      is_navigating_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      active_goal_handle_ = goal_handle;
    }
  }

  void feedback_callback(
    GoalHandleNavigate::SharedPtr,
    const std::shared_ptr<const Navigate::Feedback> feedback)
  {
    // Keeping feedback output clean so it doesn't spam the terminal while user tries to type
    RCLCPP_DEBUG(this->get_logger(), "Distance: %.2f | Angle: %.2f", 
                feedback->distance_to_goal, feedback->angle_to_goal);
  }

  void result_callback(const GoalHandleNavigate::WrappedResult & result)
  {
    is_navigating_ = false;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded! Final position: (%.2f, %.2f)", 
                    result.result->final_x, result.result->final_y);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
  }
};
}  // namespace nav_project

RCLCPP_COMPONENTS_REGISTER_NODE(nav_project::UIClient)
