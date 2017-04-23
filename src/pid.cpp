#include "pid.h"
#include "assert.h" 
#include <unistd.h>

PID::PID() {
  initialized_ = false;
}

PID::PID(double kp, double kd, double ki, double min, double max)
    : kp_(kp), kd_(kd), ki_(ki), min_(min), max_(max) {
  old_time_ = ros::Time::now();

  set_ = 0.0;
  old_pos = 0.0;
  old_velocities_[0] = 0.0;
  old_velocities_[1] = 0.0;

  integral_ = 0.0;
  initialized_ = true;
}

void PID::setGains(double kp, double kd, double ki, double min, double max) {
  kp_ = kp;
  kd_ = kd;
  ki_ = ki;
  min_ = min;
  max_ = max;
  initialized_ = true;
}

void PID::set(double setpoint) { 
  assert(!checkInitialized());
  set_ = setpoint; 
}

bool PID::checkInitialized() {
  if(initialized_ == false) {
    ROS_ERROR("Using uninitialized PID");
    exit(-1);
  }
}

double PID::get_pid(double actual) {
  assert(!checkInitialized());
  ros::Time time = ros::Time::now();
  double dt = (time - old_time_).toSec();
  old_time_ = time;
  double error = set_ - actual;
  double velocity = filterVelocity(error, old_velocities_, dt);
  return get_pidd_(error, velocity, dt);
}

double PID::get_pidd(double actual, double velocity) {
  assert(!checkInitialized());
  ros::Time time = ros::Time::now();
  double dt = (time - old_time_).toSec();
  old_time_ = time;
  double error = set_ - actual;
  return get_pidd_(error, velocity, dt);
}

void PID::reset_I() { integral_ = 0.0; }

double PID::get_pidd_(double actual, double velocity, double dt) {
  assert(!checkInitialized());
  double error = set_ - actual;
  integral_ += dt * error;
  double effort = kp_ * (set_ - actual) + kp_ * (-velocity) + ki_ * integral_;
  if(effort < min_) {
    effort = min_;
  } else if(effort > max_) {
    effort = max_;
  }
  return effort;
}

double PID::filterVelocity(double pos, double *old_vel, double dt) {
  assert(!checkInitialized());
  double vel = (pos - old_pos) / dt;
  vel = (vel + old_vel[0] + old_vel[1]) /
        3.0;  // average old values with current value
  // save old values
  old_pos = pos;
  old_vel[1] = old_vel[0];
  old_vel[0] = vel;
  return vel;
}
