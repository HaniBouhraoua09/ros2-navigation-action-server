#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_interfaces/action/navigate.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
      // We will write the navigation math here next!
  }
};
}  // namespace nav_project

RCLCPP_COMPONENTS_REGISTER_NODE(nav_project::NavigationServer)
