#include "utils.h"

Eigen::Vector3d AngvelToEulerRates(const Eigen::Vector3d& angVel, const Eigen::Vector3d& EulerAngle)
{

    double roll  = EulerAngle(0);
    double pitch = EulerAngle(1);
    double yaw   = EulerAngle(2);

    Eigen::Matrix3d T_inv;
    T_inv << cos(yaw)/cos(pitch), sin(yaw)/cos(pitch), 0,
            -sin(yaw),            cos(yaw),            0,
             cos(yaw)*tan(pitch), tan(pitch)*sin(yaw), 1;

    return (T_inv * angVel); // Euler rates
}
