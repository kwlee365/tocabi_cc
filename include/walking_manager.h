#pragma once
#include <Eigen/Dense>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "utils.h"

class WalkingManager 
{
public:
WalkingManager(RobotData &rd);

void updateContactState(const bool &local_LF_contact_, const bool &local_RF_contact_);
void getTimeInformation(const int &step_tick_, const int &trajectory_duration_);
void setControlFrequency(double &hz);
void computeWalkingMotion(const int &step_cnt, const double &step_length_x_, const double &step_length_y_, const double &step_length_yaw_, const double & foot_height);
int updateStepTick(int &step_cnt);
void updateSupportInitialState();

private:
RobotData &rd_;

void getFootTrajectory(const double &foot_height);
void getPelvTrajectory();

Eigen::Vector2d footstep_des;
Eigen::Vector2d swing_hip_pos_des;

double step_length_x = 0.0;
double step_length_y = 0.0;
double step_length_yaw = 0.0;

int step_tick = 0;
int trajectory_duration = 0;

bool local_LF_contact = true;
bool local_RF_contact = true;
bool is_support_transition = true;
};