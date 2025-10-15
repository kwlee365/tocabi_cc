#pragma once
#include <string>
#include <Eigen/Dense>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "utils.h"
#include "walking_manager.h"

class TaskManager
{
public:
  TaskManager(RobotData &rd);

  void setControlFrequency(double &hz);
  void runTestMotion(const TaskMotionType& motion_mode, const double &traj_time, const double &pelv_dist, const double &hand_dist, const double &foot_height, const double &step_duration);

private:
  void movePelvHandPose(const double& traj_time, const double& pelv_dist, const double& hand_dist);
  void moveTaichiMotion(const double& traj_time, const double& pelv_dist, const double& hand_dist, const double &foot_height);
  void bipedalWalkingController(const double &step_duration, const double &foot_height);


private:
  RobotData &rd_;
  double hz_ = 2000.0;
};
