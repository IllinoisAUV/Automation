#ifndef AUTOMATION_H
#define AUTOMATION_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <string>
#include "pid.h"

class Automation {
 public:
  Automation(std::string gains);
  static void ArmPixhawk();
  static void DisarmPixhawk();
  static void SetIMURate(int hz);

  //----------------------------------------------
  void SetRPY(double roll, double pitch, double yaw);

  void spin(float hz);
  void spinOnce();

  void SetDepth(double depth);
  double GetDepth();
  //---------------------------------------------------
 private:
  enum {
    MODE_STABILIZE = 1000,
    MODE_ALT_HOLD = 2000
  };  // ppm in uS; from ArduSub/radio.cpp

  // functions
  void gainsFromFile(std::string file);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void depthCallback(const sensor_msgs::FluidPressure::ConstPtr &msg);

  void angularSetpointCallback(const geometry_msgs::Vector3::ConstPtr &msg);
  void linearSetpointCallback(const geometry_msgs::Vector3::ConstPtr &msg);

  void armingCallback(const std_msgs::Bool::ConstPtr &msg);
  uint16_t mode_;
  uint16_t camera_tilt_;

  // node handle
  ros::NodeHandle nh;

  // pubs
  ros::Publisher rc_override_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber depth_sub_;

  ros::Subscriber linear_setpoint_sub_;
  ros::Subscriber angular_setpoint_sub_;

  ros::Subscriber arming_sub_;

  // Current values
  double pressure_;
  double roll_, pitch_, yaw_;
  double roll_dot_, pitch_dot_, yaw_dot_;

  double xdot_, ydot_, zdot_;

  PID depth_pid_;
  PID roll_pid_;
  PID pitch_pid_;
  PID yaw_pid_;
};


#endif // AUTOMATION_H
