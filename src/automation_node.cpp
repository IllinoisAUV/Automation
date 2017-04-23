#include "automation.h"
#include <signal.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "automation");

  if(argc != 2) {
    ROS_ERROR("Please specify a gains file");
    return 1;
  }
  ros::NodeHandle nh;
  Automation automation(argv[1]);
  /* Automation::ArmPixhawk(); */
  automation.spin(10);
  return 0;
}
