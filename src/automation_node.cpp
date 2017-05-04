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

  if (argc != 2) {
    ROS_ERROR("Please specify a gains file");
    return 1;
  }
  ros::NodeHandle nh;
  Automation automation(argv[1]);
  Automation::ArmPixhawk();
  ros::spinOnce();

  signal(SIGINT, mySigIntHandler);

  ros::Rate rate(20);
  for(int i=0; i < 20; i++) {
    ros::spinOnce();
    rate.sleep();
  }
  automation.setDepth(automation.getDepth() - 50.0);
  automation.setRPY(0.0, 0.0, 1.5);
  while (!g_request_shutdown) {
    automation.spinOnce();
    rate.sleep();
  }
  Automation::DisarmPixhawk();
  ros::shutdown();

  return 0;
}
