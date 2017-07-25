/*
 * File: automation_ws/src/automation.cpp
 * Author: Shubhankar Agarwal <shubhankar0109@@gmail.com>
 * Date: February 2017
 * Description: Automation the control of the bluerov.
 */
#include "automation.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/StreamRate.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <functional>

using mavros_msgs::OverrideRCIn;
using mavros_msgs::StreamRate;
using sensor_msgs::Imu;
using sensor_msgs::FluidPressure;
using mavros_msgs::SetMode;

bool Automation::armed_;
Automation::Automation() {
  imu_sub_ =
      nh.subscribe("/mavros/imu/data", 1, &Automation::imuCallback, this);
  rc_override_pub_ = nh.advertise<OverrideRCIn>("/mavros/rc/override", 1);

  angular_setpoint_sub_ =
      nh.subscribe("/vehicle/angular/setpoint", 1,
                   &Automation::angularSetpointCallback, this);
  linear_setpoint_sub_ = nh.subscribe(
      "/vehicle/linearVelocity", 1, &Automation::linearSetpointCallback, this);

  arming_sub_ =
      nh.subscribe("/vehicle/arming", 1, &Automation::armingCallback, this);

  mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  mode_ = MODE_DEPTH_HOLD;
  camera_tilt_ = 1500;

  setMode(mode_);
}

void Automation::setMode(Mode mode) {
  mode_ = mode;
  SetMode mode_cmd;
  mode_cmd.request.base_mode = 0;
  switch (mode) {
    case MODE_STABILIZE:
      mode_cmd.request.custom_mode = "STABILIZE";
      break;
    case MODE_MANUAL:
      mode_cmd.request.custom_mode = "MANUAL";
      break;
    case MODE_DEPTH_HOLD:
      mode_cmd.request.custom_mode = "ALT_HOLD";
      break;
  }

  if (mode_client_.call(mode_cmd) && mode_cmd.response.success) {
    ROS_INFO("Mode changed");
  } else {
    ROS_ERROR("Failed to change mode");
  }
}

void Automation::ArmPixhawk() {
  ROS_INFO("Arming PixHawk");
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv;
  srv.request.value = true;
  if (!client.call(srv)) {
    ROS_ERROR("Failed to arm");
  }
  armed_ = true;
}
void Automation::DisarmPixhawk() {
  ROS_INFO("Disarming PixHawk");
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv;
  srv.request.value = false;
  if (!client.call(srv)) {
    ROS_ERROR("Failed to disarm");
  }
  armed_ = false;
}

void Automation::SetIMURate(int hz) {
  ROS_INFO("Setting the IMU Rate to %d Hz", hz);
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<StreamRate>("/mavros/set_stream_rate");
  StreamRate srv;
  srv.request.stream_id = 0;  // StreamRate::STREAM_ALL;
  srv.request.message_rate = hz;
  srv.request.on_off = 0;
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call set_stream_rate service");
  }
}

void Automation::setRPY(double roll, double pitch, double yaw) {
  // Map angle to [-pi, pi]
  roll_set_ = roll - (round(roll / M_PI) * M_PI);
  pitch_set_ = pitch - (round(pitch / M_PI) * M_PI);
}

void Automation::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                   msg->orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  /* ROS_INFO("Got rpy: %f %f %f", roll_, pitch_, yaw_); */
  roll_dot_ = msg->angular_velocity.x;
  pitch_dot_ = msg->angular_velocity.y;
  yaw_dot_ = msg->angular_velocity.z;
}

void Automation::armingCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    ArmPixhawk();
  } else {
    DisarmPixhawk();
  }
}

void Automation::angularSetpointCallback(
    const geometry_msgs::Vector3::ConstPtr &msg) {
  ROS_INFO("Received angular setpoint: %f %f %f", msg->x, msg->y, msg->z);
  roll_set_ = msg->x;
  pitch_set_ = msg->y;
  yaw_dot_ = msg->z;
}

void Automation::linearSetpointCallback(
    const geometry_msgs::Vector3::ConstPtr &msg) {
  ROS_INFO("Received linear setpoint: %f %f %f", msg->x, msg->y, msg->z);
  xdot_ = msg->x;
  ydot_ = msg->y;
  zdot_ = msg->z;
}

void Automation::spin(float hz) {
  ros::Rate rate(hz);

  while (ros::ok()) {
    ros::spinOnce();
    OverrideRCIn msg;
    msg.channels[1] = angleToPpm(roll_set_);
    msg.channels[0] = angleToPpm(pitch_set_);
    msg.channels[3] = speedToPpm(yaw_dot_);

    msg.channels[5] = speedToPpm(xdot_);
    msg.channels[6] = speedToPpm(ydot_);
    msg.channels[2] = speedToPpm(zdot_);

    msg.channels[4] = mode_;
    msg.channels[7] = camera_tilt_;
    rc_override_pub_.publish(msg);
    rate.sleep();
  }
}

void Automation::setSpeed(double x, double y, double z) {
  xdot_ = x;
  ydot_ = y;
  zdot_ = z;
}

uint16_t Automation::angleToPpm(double angle) {
  // Map [-pi, pi] -> [1000, 2000]
  uint16_t ppm = (angle - (-M_PI)) / (M_PI - (-M_PI)) * (1000) + 1000;
  return ppm;
}

uint16_t Automation::speedToPpm(double speed) {
  if (speed > 1.0 || speed < -1.0) {
    ROS_ERROR("Invalid speed requested: %f", speed);
    return 1500;
  }
  return 1500 + speed * 500.0;
}

void Automation::spinOnce() {
  // Apply the PID controllers
  ros::spinOnce();
  OverrideRCIn msg;

  msg.channels[1] = angleToPpm(roll_set_);
  msg.channels[0] = angleToPpm(pitch_set_);
  msg.channels[3] = speedToPpm(yaw_dot_);

  msg.channels[5] = speedToPpm(xdot_);
  msg.channels[6] = speedToPpm(ydot_);
  msg.channels[2] = speedToPpm(zdot_);

  msg.channels[4] = mode_;
  msg.channels[7] = camera_tilt_;
  rc_override_pub_.publish(msg);
}
