#pragma once

#include <eigen3/Eigen/Core>
#include <cmath>
#include <iostream>
#include "wholebody_functions.h"

#include "task_definition.h"

class FootstepPlanner {
public:
    FootstepPlanner();
    Eigen::Vector2d planFootstep(const Eigen::Vector3d& current_hip_pos,
                                 const Eigen::Vector3d& current_base_vel,
                                 const double& step_duration,
                                 const double& vx,
                                 const double& vy,
                                 const double& wz);

private:
};