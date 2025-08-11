#pragma once

#include <eigen3/Eigen/Core>
#include <map>
#include <vector>
#include <string>
#include "wholebody_functions.h"
#include "task_definition.h"

class KinWBC {
public:
    KinWBC(int dof);

    void computeTaskSpaceKinematicWBC(
        const std::vector<std::vector<TaskInfo>>& task_hierarchy,
        const std::map<std::string, Eigen::Vector3d>& x_desired,
        const std::map<std::string, Eigen::Vector3d>& dx_desired,
        const std::map<std::string, Eigen::Vector3d>& ddx_desired,
        const std::map<std::string, Eigen::Matrix3d>& R_desired,
        const std::map<std::string, Eigen::Vector3d>& w_desired,
        const std::map<std::string, Eigen::Vector3d>& dw_desired,
        const std::map<std::string, Eigen::Vector3d>& task_Kp, 
        const std::map<std::string, Eigen::Vector3d>& task_Kv, 
        const std::map<std::string, Eigen::Vector3d>& base_ee_pos,
        const std::map<std::string, Eigen::Matrix3d>& base_ee_rot,
        const std::map<std::string, Eigen::Vector3d>& base_ee_v, 
        const std::map<std::string, Eigen::Vector3d>& base_ee_w, 
        const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_v,
        const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_w,
        const Eigen::MatrixXd& base_contact_Jac,
        const Eigen::MatrixVVd& M_inv,
        const Eigen::VectorVQd& qdot,
        Eigen::VectorVQd& dq_des,
        Eigen::VectorVQd& qdot_des,
        Eigen::VectorVQd& qddot_des);

    void computeJointSpaceKinematicWBC(
        const std::vector<std::vector<TaskInfo>>& task_hierarchy,
        const std::map<std::string, Eigen::Vector3d>& x_desired,
        const std::map<std::string, Eigen::Vector3d>& dx_desired,
        const std::map<std::string, Eigen::Vector3d>& ddx_desired,
        const std::map<std::string, Eigen::Matrix3d>& R_desired,
        const std::map<std::string, Eigen::Vector3d>& w_desired,
        const std::map<std::string, Eigen::Vector3d>& dw_desired,
        const std::map<std::string, Eigen::Vector3d>& task_Kp, 
        const std::map<std::string, Eigen::Vector3d>& task_Kv, 
        const std::map<std::string, Eigen::Vector3d>& base_ee_pos,
        const std::map<std::string, Eigen::Matrix3d>& base_ee_rot,
        const std::map<std::string, Eigen::Vector3d>& base_ee_v, 
        const std::map<std::string, Eigen::Vector3d>& base_ee_w, 
        const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_v,
        const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_w,
        const Eigen::MatrixXd& base_contact_Jac,
        const Eigen::MatrixVVd& M_inv,
        const Eigen::VectorVQd& qdot,
        Eigen::VectorVQd& dq_des,
        Eigen::VectorVQd& qdot_des,
        Eigen::VectorVQd& qddot_des);

    bool is_mode_temp_init = true;

private:
    int dof_;
    double dde_cut = 100.0;
};