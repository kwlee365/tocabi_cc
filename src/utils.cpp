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


Eigen::Vector3d getOrientationError(const Eigen::Matrix3d& current_rotation,
                                    const Eigen::Matrix3d& desired_rotation)
{
    //--- Rotation Difference
    Eigen::Matrix3d R_err; R_err.setZero();
    R_err = desired_rotation.transpose() * current_rotation;

    //--- Log Matrix
    double r11 = R_err(0, 0);
    double r22 = R_err(1, 1);
    double r33 = R_err(2, 2);
    double trace = r11 + r22 + r33;

    Eigen::Vector3d l;
    l << R_err(2, 1) - R_err(1, 2),
         R_err(0, 2) - R_err(2, 0),
         R_err(1, 0) - R_err(0, 1);

    double norm_l = l.norm();
    double theta = std::atan2(norm_l, trace - 1.0);

    Eigen::Vector3d phi; phi.setZero();
    const double epsilon = 1e-6;

    
    if (R_err.isIdentity() == true) // Case 1: R = I (no rotation)
    {
        phi.setZero();
    }
    else if (R_err.isDiagonal() == true)    // Case 2: R is 180-degree flip (diagonal rotation matrix)
    {
        phi << (M_PI / 2.0)* (r11 + 1),
               (M_PI / 2.0)* (r22 + 1),
               (M_PI / 2.0)* (r33 + 1);
    }
    // Case 3: General case
    else
    {
        phi = theta * l / norm_l;
    }

    return phi;
}