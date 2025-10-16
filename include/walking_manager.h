#pragma once
#include <Eigen/Dense>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "utils.h"
#include <deque>
#include <iomanip>

class WalkingManager 
{
public:
WalkingManager(RobotData &rd);

void computeWalkingMotion();

//---Setter
void updateContactState(const bool &local_LF_contact_, const bool &local_RF_contact_);
void setControlFrequency(const double &hz);
void setCenterOfMassHeight(const double &com_height_);
void setWalkingParameter(const double &step_length_, const double &foot_yaw_angle_, const double &foot_height_);
void setTimeInformation(const int &step_tick_, const int &trajectory_duration_);
void setStepDuration(const double &step_duration_);
void setTransferDuration(const double &transfer_duration_);

private:
RobotData &rd_;
double hz_;

void updateStepTick();
void updateSupportInitialState();
void calcFootstepQueue();
void calcCapturePointQueue();
void getFootTrajectory();
void getComTrajectory();

Eigen::Vector2d footstep_des;

const int preview_idx = 3;
Eigen::Vector2d step_command;
std::deque<Eigen::Vector2d> step_queue;
std::deque<Eigen::Vector2d> cp_end_queue;
double step_length    = 0.0;
double step_width     = 0.0;
double foot_yaw_angle = 0.0;
double foot_height = 0.0;

int step_tick = 0;
double step_time = 0.0;
int step_cnt = 0;
double step_duration = 0.0;
double transfer_duration = 0.0;
double trajectory_duration = 0;

double com_height = 0.0;
double wn = 0.0;
double T = 0.0;

bool local_LF_contact = true;
bool local_RF_contact = true;

bool is_support_transition = false;
bool is_footstep_update = false;
bool is_cp_eos_update = false;
};