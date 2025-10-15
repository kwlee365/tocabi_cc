#pragma once
#include <string>
#include <Eigen/Dense>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "utils.h"

class ControlManager
{
public:
    ControlManager(RobotData &rd);

    void setRobotModel();
    void update();
    void saveInitialState();

private:
    // Depedencies
    RobotData &rd_;
    RigidBodyDynamics::Model model_;  

    void contactStateMachine();
    void mapGlobalToBase();
    void mapBaseToSupport();
    void updateDynamics();
    void updateContact();

    Eigen::Vector3d base_pos; 
    Eigen::Matrix3d base_rot; 
    
};