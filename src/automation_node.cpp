#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include "automation.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig) { g_request_shutdown = 1; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "automation");

  ros::NodeHandle nh;
  Automation automation;
  Automation::ArmPixhawk();
  ros::spinOnce();

  signal(SIGINT, mySigIntHandler);

  ros::Rate rate(20);
  for(int i=0; i < 20; i++) {
    ros::spinOnce();
    rate.sleep();
  }
  // Set roll and pitch setpoints, and yaw rate of change
  automation.setRPY(0.0, 0.0, 0.0);
  // Set x, y, z rate changes
  automation.setSpeed(0.0, 0.0, 0.0);
  while (!g_request_shutdown) {
    automation.spinOnce();
    rate.sleep();
  }
  Automation::DisarmPixhawk();
  ros::shutdown();

  return 0;
}
