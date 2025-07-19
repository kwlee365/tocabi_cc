#pragma once

#include <eigen3/Eigen/Core>
#include <map>
#include <vector>
#include <string>
#include "wholebody_functions.h"

enum class TaskType { Position, Orientation };

struct TaskInfo {
    std::string link_name;
    TaskType type;
};

class KinWBC {
public:
    KinWBC(int dof);

    void computeKinematicWBC(
        const std::vector<std::vector<TaskInfo>>& task_hierarchy,
        const std::map<std::string, Eigen::Vector3d>& x_desired,
        const std::map<std::string, Eigen::Vector3d>& dx_desired,
        const std::map<std::string, Eigen::Vector3d>& ddx_desired,
        const std::map<std::string, Eigen::Matrix3d>& R_desired,
        const std::map<std::string, Eigen::Vector3d>& w_desired,
        const std::map<std::string, Eigen::Vector3d>& dw_desired,
        const std::map<std::string, Eigen::Vector3d>& base_ee_pos,
        const std::map<std::string, Eigen::Matrix3d>& base_ee_rot,
        const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_v,
        const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_w,
        const Eigen::MatrixXd& base_contact_Jac,
        const Eigen::MatrixVVd& M,
        Eigen::VectorXd& q_des,
        Eigen::VectorXd& qdot_des,
        Eigen::VectorXd& qddot_des);

    Eigen::Vector3d orientationError(const Eigen::Matrix3d& R, const Eigen::Matrix3d& R_des);
    static Eigen::MatrixXd pinv_SVD(const Eigen::MatrixXd& A, double tolerance = 1e-6);

    bool is_mode_temp_init = true;

private:
    int dof_;
};
