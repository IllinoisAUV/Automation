#ifndef AUTOMATION_H
#define AUTOMATION_H

#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
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
  // Set desired roll pitch and yaw in radians
  void setRPY(double roll, double pitch, double yaw);
  // Set speed as a percent
  void setSpeed(double x, double y, double z);

  void spin(float hz);
  void spinOnce();

  void setDepth(double depth);
  double getDepth();
  //---------------------------------------------------
 private:
  typedef enum {
    MODE_STABILIZE = 1000,
    MODE_MANUAL = 1500,
    MODE_DEPTH_HOLD = 2000,
  } Mode;  // ppm in uS; from ArduSub/radio.cpp

  // functions
  void gainsFromFile(std::string file);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void depthCallback(const sensor_msgs::FluidPressure::ConstPtr &msg);

  void setMode(Mode mode);

  void angularSetpointCallback(const geometry_msgs::Vector3::ConstPtr &msg);
  void linearSetpointCallback(const geometry_msgs::Vector3::ConstPtr &msg);

  void armingCallback(const std_msgs::Bool::ConstPtr &msg);

  uint16_t angleToPpm(double angle);
  uint16_t speedToPpm(double speed);
  Mode mode_;
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

  ros::ServiceClient mode_client_;

  // Current values
  double pressure_;
  double roll_, pitch_, yaw_;
  double roll_dot_, pitch_dot_, yaw_dot_;

  double xdot_, ydot_, zdot_;

  PID depth_pid_;
  double roll_set_;
  double pitch_set_;
  PID roll_pid_;
  PID pitch_pid_;
  PID yaw_pid_;
};

#endif  // AUTOMATION_H
