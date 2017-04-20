/*
 * File: automation_ws/src/automation.cpp
 * Author: Shubhankar Agarwal <shubhankar0109@@gmail.com>
 * Date: February 2017
 * Description: Automation the control of the bluerov.
 */

#include <signal.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/StreamRate.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <unistd.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <functional>
#include <mavros_msgs/Mavlink.h>
#include <stdio.h>

#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class Automation {
  public:
    Automation();
    static void ArmPixhawk();
    static void DisarmPixhawk();
    static void SetIMURate(int hz);

    void shutdown(int signum);

    //----------------------------------------------
    void Roll( float turn_amount );
    void Throttle( float amount );
    void Depth( float amount );
    void Yaw( uint16_t amount );
    void rc_override(int forward, int left, int throttle);
    void turn();
    //---------------------------------------------------
  private:

    enum {MODE_STABILIZE=1000, MODE_ALT_HOLD=2000}; // ppm in uS; from ArduSub/radio.cpp
    // functions
    void quatCallback(const sensor_msgs::Imu::ConstPtr &msg);

    uint16_t mode;
    uint16_t camera_tilt;

    // node handle
    ros::NodeHandle nh;

    // pubs
    ros::Publisher rc_override_pub;
    ros::Publisher joy_pub;
    ros::Subscriber quat_sub;
};

Automation::Automation() {
  quat_sub = nh.subscribe("/mavros/imu/data", 100, &Automation::quatCallback, this);
  rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

  mode = MODE_STABILIZE;
  /* mode = MODE_ALT_HOLD; */
  camera_tilt = 1500;

  // f310 axes (from): [left X, left Y, LT, right X, right Y, RT, pad L/R, pad U/D]
  // f310 buttons (from): [A, B, X, Y LB, RB, BACK, START, POWER, left stick, right stick click]
}



void Automation::ArmPixhawk() {
  ROS_INFO("Arming PixHawk");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv;
  srv.request.value = true;
  if(!client.call(srv)) {
      ROS_ERROR("Failed to arm");
  }
}
void Automation::DisarmPixhawk() {
  ROS_INFO("Disarming PixHawk");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv;
  srv.request.value = false;
  if(!client.call(srv)) {
      ROS_ERROR("Failed to disarm");
  }
}

void Automation::SetIMURate(int hz) {
  ROS_INFO("Setting the IMU Rate to %d Hz", hz);
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
  mavros_msgs::StreamRate srv;
  srv.request.stream_id = 0; // mavros_msgs::StreamRate::STREAM_ALL;
  srv.request.message_rate = hz;
  srv.request.on_off = 0;
  if(!client.call(srv)) {
    ROS_ERROR("Failed to call set_stream_rate service");
  }
}

void Automation::Roll( float turn_amount )
{
  ROS_INFO("Change Roll");
  sensor_msgs::Joy out;
  out.axes = std::vector<float>(8, 0);
  out.buttons = std::vector<int32_t>(11, 0);

  // TODO : check if the right button
  out.axes[3] = turn_amount;

  joy_pub.publish(out);
}

void Automation::Throttle( float amount  )
{
	ROS_INFO("Throttle");
	sensor_msgs::Joy out;
	out.axes = std::vector<float>(8, 0);
	out.buttons = std::vector<int32_t>(11, 0);

	// TODO : check if the right button
	out.axes[1] = amount;

	joy_pub.publish(out);
}

// positive value ascend and negetive desend
void Automation::Depth( float amount )
{
	ROS_INFO( " Change Depth ");
	sensor_msgs::Joy out;
	out.axes = std::vector<float>(8,0);
	out.buttons = std::vector<int32_t>(11,0);

	// TODO : check if the right button
	out.axes[4] = amount;
	joy_pub.publish(out);

}

void Automation::Yaw( uint16_t amount )
{
	ROS_INFO("Change Yaw");
	mavros_msgs::OverrideRCIn msg;

	msg.channels[5] = -1; // forward  (x)
    msg.channels[6] = -1; // strafe   (y)
	msg.channels[2] = -1; // throttle (z)

	msg.channels[1] = -1; // roll     (wx)
	msg.channels[0] = -1; // pitch    (wy)
	msg.channels[3] = 1500 + amount; // yaw      (wz)

	msg.channels[4] = mode; // mode
	msg.channels[7] = -1; // camera tilt

    ROS_INFO("/mavros/rc/override: %d %d %d", msg.channels[5], msg.channels[6], msg.channels[2]);
	rc_override_pub.publish(msg);

}

void Automation::turn() {
	mavros_msgs::OverrideRCIn msg;

	msg.channels[5] = 1500 + 100; // forward  (x)
    msg.channels[6] = 1500; // strafe   (y)
	msg.channels[2] = 1500+100; // throttle (z)

	msg.channels[1] = 1500; // roll     (wx)
	msg.channels[0] = 1500; // pitch    (wy)
	msg.channels[3] = 1500 + 400; // yaw      (wz)

	msg.channels[4] = mode; // mode
	msg.channels[7] = camera_tilt; // camera tilt

	rc_override_pub.publish(msg);
}
void Automation::rc_override(int forward, int left, int throttle)
{
    ROS_INFO("Sending rc_override values");
	mavros_msgs::OverrideRCIn msg;

	msg.channels[5] = 1500 + forward; // forward  (x)
    msg.channels[6] = 1500; // strafe   (y)
	msg.channels[2] = 1500 + throttle; // throttle (z)

	msg.channels[1] = 1500; // roll     (wx)
	msg.channels[0] = 1500; // pitch    (wy)
	msg.channels[3] = 1500 + left; // yaw      (wz)

	msg.channels[4] = mode; // mode
	msg.channels[7] = camera_tilt; // camera tilt

    ROS_INFO("/mavros/rc/override: %d %d %d", msg.channels[5], msg.channels[6], msg.channels[2]);
	rc_override_pub.publish(msg);
}


void Automation::quatCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  ROS_INFO("Quat: %f %f %f %f", msg->orientation.x, msg->orientation.y,
	   msg->orientation.z, msg->orientation.w);
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
		   msg->orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("RPY: %f, %f, %f", roll, pitch, yaw);
}

struct termios orig_termios;

void disableRawMode() {
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enableRawMode() {
  tcgetattr(STDIN_FILENO, &orig_termios);
  atexit(disableRawMode);

  struct termios raw = orig_termios;
  raw.c_lflag &= ~(ECHO | ICANON);

  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "automation");

  ros::NodeHandle nh;
  Automation automation;
  Automation::ArmPixhawk();
  signal(SIGINT, mySigIntHandler);


  enableRawMode();
  // Make stdin nonblocking
  int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);


  ros::Rate r(5);
  while(!g_request_shutdown) {
      automation.turn();
      /* char c; */
      /* if(read(STDIN_FILENO, &c, 1) == 1) { */
      /*     ROS_INFO("Got %c", c); */
      /*     switch(c) { */
      /*         case 'w': */
      /*             automation.rc_override(200, 0, 0); */
      /*             break; */
      /*         case 'a': */
      /*             automation.rc_override(0, 200, 0); */
      /*             /1* automation.Yaw(-200); *1/ */
      /*             break; */
      /*         case 's': */
      /*             automation.rc_override(-200, 0, 0); */
      /*             break; */
      /*         case 'd': */
      /*             /1* automation.Yaw(-200); *1/ */
      /*             automation.rc_override(0, -200, 0); */
      /*             break; */
      /*         case 'k': */
      /*             automation.rc_override(0, 0, 200); */
      /*             break; */
      /*         case 'j': */
      /*             automation.rc_override(0, 0, -200); */
      /*             break; */
      /*         case 'q': */
      /*             g_request_shutdown = 1; */
      /*             break; */
      /*     } */
      /*     while (getchar() != EOF); */
      /* } else { */
      /*     automation.rc_override(0, 0, 0); */
      /*     ROS_INFO("No character received"); */
      /* } */

      /* automation.rc_override(); */

      ros::spinOnce();
      r.sleep();
  }
  Automation::DisarmPixhawk();
  ros::shutdown();
  return 0;
}
