#ifndef COPTO_PID__PID_COMPONENT_HPP_
#define COPTO_PID__PID_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COPTO_PID_PID_COMPONENT_EXPORT __attribute__((dllexport))
#define COPTO_PID_PID_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define COPTO_PID_PID_COMPONENT_EXPORT __declspec(dllexport)
#define COPTO_PID_PID_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef COPTO_PID_PID_COMPONENT_BUILDING_DLL
#define COPTO_PID_PID_COMPONENT_PUBLIC COPTO_PID__PID_COMPONENT_EXPORT
#else
#define COPTO_PID_PID_COMPONENT_PUBLIC COPTO_PID__PID_COMPONENT_IMPORT
#endif
#define COPTO_PID__PID_COMPONENT_PUBLIC_TYPE COPTO_PID__PID_COMPONENT_PUBLIC
#define COPTO_PID_PID_COMPONENT_LOCAL
#else
#define COPTO_PID_PID_COMPONENT_EXPORT __attribute__((visibility("default")))
#define COPTO_PID_PID_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define COPTO_PID_PID_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define COPTO_PID_PID_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define COPTO_PID_PID_COMPONENT_PUBLIC
#define COPTO_PID_PID_COMPONENT_LOCAL
#endif
#define COPTO_PID_PID_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace copto_pid
{
class PIDComponent : public rclcpp::Node
{
public:
  COPTO_PID_PID_COMPONENT_PUBLIC
  explicit PIDComponent(const rclcpp::NodeOptions & options);

private:
  void IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMUsubscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Posepublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace copto_pid

#endif  // COPTO_PID__PID_COMPONENT_HPP_