#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_interfaces/action/navigate.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

// We need this to convert the quaternion orientation into yaw (theta)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace nav_project
{
class NavigationServer : public rclcpp::Node
{
public:
  using Navigate = nav_interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

  explicit NavigationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("navigation_server", options)
  {
    // 1. Initialize the Action Server
    this->action_server_ = rclcpp_action::create_server<Navigate>(
      this,
      "navigate_robot",
      std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1));

    // 2. Setup Publisher for velocity
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 3. Setup Subscriber for current position
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&NavigationServer::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Navigation Server Component is ready!");
  }

private:
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables to store the robot's current state
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;

  // --- ODOMETRY CALLBACK ---
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Convert quaternion to Euler angles to get the yaw (theta)
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
  }

  // --- ACTION SERVER CALLBACKS ---
  
  // 1. Decide whether to accept or reject a new goal
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: x=%.2f, y=%.2f", goal->target_x, goal->target_y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 2. Decide whether to accept a cancel request
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 3. What to do once the goal is accepted
  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    // We launch the execution in a separate thread to avoid blocking the ROS executor!
    std::thread{std::bind(&NavigationServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  
  // --- THE CONTROL LOOP ---
  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    rclcpp::Rate loop_rate(10); // Run the control loop at 10 Hz
    
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Navigate::Feedback>();
    auto result = std::make_shared<Navigate::Result>();
    auto twist_msg = geometry_msgs::msg::Twist();

    double distance_tolerance = 0.1;
    double angle_tolerance = 0.05;

    while (rclcpp::ok()) {
      // 1. Check if the client requested to cancel the action
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->final_x = current_x_;
        result->final_y = current_y_;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled by user!");
        
        // Stop the robot immediately
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(twist_msg);
        return;
      }

      // 2. Calculate errors (distance and angle)
      double diff_x = goal->target_x - current_x_;
      double diff_y = goal->target_y - current_y_;
      double distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
      double target_angle = std::atan2(diff_y, diff_x);
      
      // Normalize the angle difference to stay between -pi and pi
      double angle_diff = target_angle - current_yaw_;
      while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

      // 3. Publish live feedback to the client
      feedback->distance_to_goal = distance;
      feedback->angle_to_goal = angle_diff;
      goal_handle->publish_feedback(feedback);

      // 4. Proportional Control Logic
      if (distance > distance_tolerance) {
        // If the robot is facing the wrong way, rotate first
        if (std::abs(angle_diff) > 0.2) {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.8 * angle_diff; 
        } else {
            // Move forward and adjust steering
            twist_msg.linear.x = 0.5 * distance;
            twist_msg.angular.z = 0.8 * angle_diff;
        }

        // Cap the maximum speeds so the robot doesn't fly off the map
        if (twist_msg.linear.x > 1.0) twist_msg.linear.x = 1.0;
        if (twist_msg.angular.z > 1.0) twist_msg.angular.z = 1.0;
        if (twist_msg.angular.z < -1.0) twist_msg.angular.z = -1.0;

        cmd_vel_pub_->publish(twist_msg);
      } else {
        // We reached the (x, y) target! Now align with the final target_theta
        double final_angle_diff = goal->target_theta - current_yaw_;
        while (final_angle_diff > M_PI) final_angle_diff -= 2.0 * M_PI;
        while (final_angle_diff < -M_PI) final_angle_diff += 2.0 * M_PI;

        if (std::abs(final_angle_diff) > angle_tolerance) {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.8 * final_angle_diff;
            cmd_vel_pub_->publish(twist_msg);
        } else {
            // We are perfectly positioned and aligned. Break the loop!
            break;
        }
      }

      // Sleep to maintain our 10 Hz loop rate
      loop_rate.sleep();
    }

    // 5. Goal Successfully Reached
    if (rclcpp::ok()) {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(twist_msg);

      result->success = true;
      result->final_x = current_x_;
      result->final_y = current_y_;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Target successfully reached!");
    }
  }
};
}  // namespace nav_project

RCLCPP_COMPONENTS_REGISTER_NODE(nav_project::NavigationServer)
