#pragma once
#include <Eigen/Dense>
#include "math_type_define.h"
#include "task_definition.h"

Eigen::Vector3d AngvelToEulerRates(const Eigen::Vector3d& angVel, const Eigen::Vector3d& EulerAngle);
Eigen::Vector3d getOrientationError(const Eigen::Matrix3d& current_rotation, const Eigen::Matrix3d& desired_rotation);
inline std::string contactIndicatorToString(const ContactIndicator& mode)
{
    switch (mode)
    {
        case ContactIndicator::DoubleSupport:      return "DoubleSupport";
        case ContactIndicator::LeftSingleSupport:  return "LeftSingleSupport";
        case ContactIndicator::RightSingleSupport: return "RightSingleSupport";
        default:                                   return "Unknown";
    }
}