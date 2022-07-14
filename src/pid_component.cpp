#include <cmath>
#include <chrono>
#include <copto_pid/pid_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace  std::chrono_literals;
namespace copto_pid
{
PIDComponent::PIDComponent(const rclcpp::NodeOptions & options) : Node("copto_pid_node", options)
{
  POSEsubscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/copto/pose", 10, std::bind(&PIDComponent::POSEtopic_callback, this, std::placeholders::_1));

  JOYsubscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&PIDComponent::JOYtopic_callback, this, std::placeholders::_1));

  Posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/copto/pose", 1);

  timer_ = this->create_wall_timer(10ms, std::bind(&PIDComponent::update, this));
}

void PIDComponent::POSEtopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  tf2::Quaternion quat;  
  quat = mag-> pose.pose.orientation;
  tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
  yawrate_ = yaw_old - yaw_;
  yaw_old = yaw_;
}

void PIDComponent::JOYtopic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  ctl_thrott = msg->axis[1] * MAX_THROTT;
  ctl_yawrate = msg->axis[0] * MAX_YAWRATE;
  ctl_pitch = msg->axis[5] * MAX_PITCH;
  ctl_roll = msg->axis[6] * MAX_ROLL;
}

void PIDComponent::update()
{
  double e_pitch_new, e_roll_new, e_thrott_new, e_yawrate_new;
  double u_pitch, u_roll, u_thrott, u_yawrate;

  // e_thrott_new = thrott_-ctl_thrott;
  e_thrott_new = 0;
  e_yawrate_new = yawrate_-ctl_yawrate;
  e_pitch_new = pitch_-ctl_pitch;
  e_roll_new = roll_-ctl_roll;

  u_thrott = Kp_t*e_thrott_new + Kd_t*(e_thrott_old-e_thrott_new)/dt;
  u_yawrate = Kp_y*e_yawrate_new + Kd_y*(e_yawrate_old-e_yawrate_new)/dt;
  u_roll = Kp_r*e_roll_new + Kd_r*(e_roll_old-e_roll_new)/dt;
  u_pitch = Kp_p*e_pitch_new + Kd_p*(e_pitch_old-e_pitch_new)/dt;

  e_thrott_old = e_thrott_new;
  e_yawrate_old = e_yawrate_new;
  e_pitch_old = e_pitch_new;
  e_roll_old = e_roll_new;
}

}  // namespace copto_pid

RCLCPP_COMPONENTS_REGISTER_NODE(copto_pid::PIDComponent)