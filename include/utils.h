#pragma once
#include <Eigen/Dense>
#include "math_type_define.h"
#include "task_definition.h"

Eigen::Vector3d AngvelToEulerRates(const Eigen::Vector3d& angVel, const Eigen::Vector3d& EulerAngle);
Eigen::Vector3d getOrientationError(const Eigen::Matrix3d& current_rotation, const Eigen::Matrix3d& desired_rotation);
Eigen::Matrix3d hat(const Eigen::Vector3d& v);
Eigen::Vector3d vee(const Eigen::Matrix3d& M);
double cubicBezierPolynomial(double current_time, double start_time, double end_time, double p_init, double p_mid, double p_end);
double cubicDotBezierPolynomial(double current_time, double start_time, double end_time, double p_init, double p_mid, double p_end);
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