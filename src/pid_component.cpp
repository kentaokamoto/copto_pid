#include <cmath>
#include <chrono>
#include <copto_pid/pid_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


using namespace  std::chrono_literals;
namespace copto_pid
{
PIDComponent::PIDComponent(const rclcpp::NodeOptions & options) : Node("copto_pid_node", options)
{
  POSEsubscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/copto/pose", 10, std::bind(&PIDComponent::POSEtopic_callback, this, std::placeholders::_1));

  JOYsubscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&PIDComponent::JOYtopic_callback, this, std::placeholders::_1));

  CTLpublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/copto/ctl_val", 1);

  timer_ = this->create_wall_timer(10ms, std::bind(&PIDComponent::update, this));
}

void PIDComponent::getEulerRPY(const geometry_msgs::msg::Quaternion q, double &roll, double &pitch, double &yaw)
{
  yaw = atan2((2*q.x*q.y+2*q.w*q.z),(2*q.w-1+2*pow(q.x,2)));
  roll = atan2((2*q.y*q.z+2*q.w*q.x),(2*q.w-1+2*pow(q.z,2)));
  pitch = asin(2*q.w*q.y-2*q.x*q.z);
}

void PIDComponent::POSEtopic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::Quaternion quat;  
  quat = msg-> pose.pose.orientation;
  getEulerRPY(quat, yaw_, pitch_, roll_);
  yawrate_ = yaw_old - yaw_;
  yaw_old = yaw_;
}

void PIDComponent::JOYtopic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  ctl_thrott = msg->axes[3] * MAX_THROTT;
  ctl_yawrate = msg->axes[0] * MAX_YAWRATE;
  ctl_pitch = msg->axes[5] * MAX_PITCH;
  ctl_roll = msg->axes[6] * MAX_ROLL;
}

void PIDComponent::update()
{
  double e_pitch_new, e_roll_new, e_yawrate_new;

  // culc error
  e_yawrate_new = yawrate_-ctl_yawrate;
  e_pitch_new = pitch_-ctl_pitch;
  e_roll_new = roll_-ctl_roll;

  // roll pitch yaw thrust
  std_msgs::msg::Float32MultiArray ctl_val;
  ctl_val.data[3] = ctl_thrott;
  // pose pd control
  ctl_val.data[2] = Kp_y*e_yawrate_new + Kd_y*(e_yawrate_old-e_yawrate_new)/dt;
  ctl_val.data[0] = Kp_r*e_roll_new + Kd_r*(e_roll_old-e_roll_new)/dt;
  ctl_val.data[1] = Kp_p*e_pitch_new + Kd_p*(e_pitch_old-e_pitch_new)/dt;

  CTLpublisher_->publish(ctl_val);
  e_yawrate_old = e_yawrate_new;
  e_pitch_old = e_pitch_new;
  e_roll_old = e_roll_new;
}

}  // namespace copto_pid

RCLCPP_COMPONENTS_REGISTER_NODE(copto_pid::PIDComponent)