/*
 * File: automation_ws/src/automation.cpp
 * Author: Shubhankar Agarwal <shubhankar0109@@gmail.com>
 * Date: February 2017
 * Description: Automation the control of the bluerov.
 */
#include "automation.h"

#include <errno.h>
#include <fcntl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/StreamRate.h>
#include <nav_msgs/Odometry.h>
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
using nav_msgs::Odometry;

Automation::Automation(std::string gains) {
  odom_sub_ =
      nh.subscribe("/odometry/filtered", 1, &Automation::odomCallback, this);
  depth_sub_ = nh.subscribe("/mavros/imu/atm_pressure", 1,
                            &Automation::depthCallback, this);
  rc_override_pub_ = nh.advertise<OverrideRCIn>("/mavros/rc/override", 1);

  angular_setpoint_sub_ = nh.subscribe("/vehicle/angular/goal", 1,
                               &Automation::angularSetpointCallback, this);
  linear_setpoint_sub_ = nh.subscribe("/vehicle/linearVelocity/goal", 1,
                               &Automation::linearSetpointCallback, this);

  arming_sub_ = nh.subscribe("/vehicle/arming", 1, &Automation::armingCallback, this);

  mode_ = MODE_STABILIZE;
  camera_tilt_ = 1500;

  gainsFromFile(gains);
}

void Automation::gainsFromFile(std::string file) {
  FILE *fd = fopen(file.c_str(), "r");
  if (fd == NULL) {
    perror("Failed to open gains file");
    exit(-1);
  }

  float kp, kd, ki, min, max;

  fscanf(fd, "Pitch\nKp: %f\nKd: %f\nKi: %f\nMin: %f\nMax: %f\n", &kp, &kd, &ki,
         &min, &max);
  printf("Pitch: %f %f %f %f %f\n", kp, kd, ki, min, max);
  pitch_pid_.setGains(kp, kd, ki, min, max);
  fscanf(fd, "Roll\nKp: %f\nKd: %f\nKi: %f\nMin: %f\nMax: %f\n", &kp, &kd, &ki,
         &min, &max);
  printf("Roll: %f %f %f %f %f\n", kp, kd, ki, min, max);
  roll_pid_.setGains(kp, kd, ki, min, max);
  fscanf(fd, "Roll\nKp: %f\nKd: %f\nKi: %f\nMin: %f\nMax: %f\n", &kp, &kd, &ki,
         &min, &max);
  printf("Yaw: %f %f %f %f %f\n", kp, kd, ki, min, max);
  yaw_pid_.setGains(kp, kd, ki, min, max);

  fscanf(fd, "Depth\nKp: %f\nKd: %f\nKi: %f\nMin: %f\nMax: %f\n", &kp, &kd, &ki,
         &min, &max);
  printf("Depth: %f %f %f %f %f\n", kp, kd, ki, min, max);
  depth_pid_.setGains(kp, kd, ki, min, max);
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

void Automation::SetRPY(double roll, double pitch, double yaw) {
  roll_pid_.set(roll);
  pitch_pid_.set(pitch);
  yaw_pid_.set(yaw);
}

void Automation::depthCallback(
    const sensor_msgs::FluidPressure::ConstPtr &msg) {
  pressure_ = msg->fluid_pressure;
}

void Automation::odomCallback(const Odometry::ConstPtr &msg) {
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  roll_dot_ = msg->twist.twist.angular.x;
  pitch_dot_ = msg->twist.twist.angular.y;
  yaw_dot_ = msg->twist.twist.angular.z;
}

void Automation::armingCallback(const std_msgs::Bool::ConstPtr &msg) {
  if(msg->data) {
    ArmPixhawk();
  } else {
    DisarmPixhawk();
  }
}

void Automation::angularSetpointCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
  ROS_INFO("Received angular setpoint: %f %f %f", msg->x, msg->y, msg->z);
  roll_ = msg->x;
  pitch_ = msg->y;
  yaw_ = msg->z;
}

void Automation::linearSetpointCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
  ROS_INFO("Received linear setpoint: %f %f %f", msg->x, msg->y, msg->z);
  xdot_ = msg->x;
  ydot_ = msg->y;
  zdot_ = msg->z;
}

void Automation::spin(float hz) {
  ros::Rate rate(hz);
  ROS_INFO("Entering loop");

  while (ros::ok()) {
    // Apply the PID controllers
    ros::spinOnce();
    OverrideRCIn msg;
    msg.channels[1] = 1500 + roll_pid_.get_pidd(roll_, roll_dot_);
    msg.channels[0] = 1500 + pitch_pid_.get_pidd(pitch_, pitch_dot_);
    msg.channels[3] = 1500 + yaw_pid_.get_pidd(yaw_, yaw_dot_);

    msg.channels[5] = 1500 + xdot_;
    msg.channels[6] = 1500 + ydot_;
    msg.channels[2] = 1500 + zdot_ + depth_pid_.get_pid(pressure_);

    msg.channels[4] = mode_;
    msg.channels[7] = camera_tilt_;
    rc_override_pub_.publish(msg);
    rate.sleep();
  }
}
