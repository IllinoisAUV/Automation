#ifndef PID_H
#define PID_H
#include <ros/ros.h>

class PID {
 public:
  PID();
  PID(double kp, double kd, double ki, double min, double max);
  void setGains(double kp, double kd, double ki, double min, double max);
  void set(double setpoint);
  double get_pid(double actual);
  double get_pidd(double actual, double velocity);
  void reset_I();

 private:
  double get_pidd_(double actual, double velocity, double dt);

  double filterVelocity(double pos, double *old_vel, double dt);

  bool checkInitialized();

  double set_;

  double old_pos;
  double old_velocities_[2];

  ros::Time old_time_;

  double kp_, kd_, ki_;
  double min_, max_;

  double integral_;
  double velocity_;
  bool initialized_;


};
#endif  // PID_H
