#include <cmath>
#include <chrono>
#include <copto_pid/pid_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace  std::chrono_literals;
namespace copto_pid
{
PIDComponent::PIDComponent(const rclcpp::NodeOptions & options) : Node("copto_pid_node", options)
{
  IMUsubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/copto/imu", 10, std::bind(&PIDComponent::IMUtopic_callback, this, std::placeholders::_1));

  Posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/copto/pose", 1);

  timer_ = this->create_wall_timer(10ms, std::bind(&PIDComponent::update, this));
}

void PIDComponent::IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imutimestamp = msg->header.stamp;
  u(0) = msg->linear_acceleration.x;
  u(1) = msg->linear_acceleration.y;
  u(2) = msg->linear_acceleration.z;
  u(3) = msg->angular_velocity.x;
  u(4) = msg->angular_velocity.y;
  u(5) = msg->angular_velocity.z;

  am << u(0), u(1), u(2);
}

}  // namespace copto_pid

RCLCPP_COMPONENTS_REGISTER_NODE(copto_pid::PIDComponent)