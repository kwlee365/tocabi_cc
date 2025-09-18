#pragma once

#include <eigen3/Eigen/Core>
#include <map>
#include <vector>
#include <string>
#include "wholebody_functions.h"
#include "task_definition.h"
#include "utils.h"

class KinWBC {
public:
    KinWBC(int dof);

    void computeTaskSpaceKinematicWBC(
        const std::vector<std::vector<TaskInfo>> &task_hierarchy,
        const ContactIndicator& contactMode,
        const std::map<std::string, Eigen::Vector3d> &x_desired, const std::map<std::string, Eigen::Vector3d> &dx_desired, const std::map<std::string, Eigen::Vector3d> &ddx_desired,
        const std::map<std::string, Eigen::Matrix3d> &R_desired, const std::map<std::string, Eigen::Vector3d> &w_desired, const std::map<std::string, Eigen::Vector3d> &dw_desired,
        const std::map<std::string, Eigen::Vector3d> &task_pos_Kp, const std::map<std::string, Eigen::Vector3d> &task_ori_Kp,
        const std::map<std::string, Eigen::Vector3d> &base_ee_pos, const std::map<std::string, Eigen::Matrix3d> &base_ee_rot,
        const std::map<std::string, Eigen::Vector3d> &base_ee_v, const std::map<std::string, Eigen::Vector3d> &base_ee_w,
        const std::map<std::string, Eigen::Matrix3Vd> &base_Jac_v, const std::map<std::string, Eigen::Matrix3Vd> &base_Jac_w, const Eigen::MatrixXd &base_contact_Jac, 
        const Eigen::VectorVQd &qdot, Eigen::VectorVQd &qdot_des);

    bool is_gradhess_init_ = true;
    bool is_filter_init_ = true;
    bool is_cannot_solve_qp_init_ = true;

    void safetyFilter(Eigen::VectorVQd &qdot_des, const Eigen::VectorVQd &q,
                      const Eigen::VectorQd &q_pos_l_lim, const Eigen::VectorQd &q_pos_h_lim, 
                      const Eigen::VectorQd &q_vel_l_lim, const Eigen::VectorQd &q_vel_h_lim);
    CQuadraticProgram QP_safety_filter;
    void calcCostHess();
    void calcCostGrad(const Eigen::VectorVQd& qdot_des);
    void calcEqualityConstraint();
    void calcInequalityConstraint(const Eigen::VectorVQd& q_, const Eigen::VectorQd& q_pos_l_lim_, const Eigen::VectorQd& q_pos_h_lim_, const Eigen::VectorQd& q_vel_l_lim_, const Eigen::VectorQd& q_vel_h_lim_);
    void checkGradHessSize();
    
    void getReachabilityConstraints(const std::vector<Eigen::MatrixXd> &J_reachability_, const std::vector<double> &h_reachability_);
    std::vector<Eigen::MatrixXd> grad_reachability_;       
    std::vector<double>          cbf_reachability_;    

    Eigen::MatrixXd Hess; 
    Eigen::VectorXd grad;
    Eigen::MatrixXd A_const;   
    Eigen::VectorXd lbA_const;
    Eigen::VectorXd ubA_const;
    std::vector<ConstraintMatrix> constraints_;
    int total_num_state = 0;
    int total_num_constraints = 0;

    Eigen::VectorVQd qdot_safety;

private:
    int dof_;
    double dt_ = 5e-4; 
    ContactIndicator contact_mode_;
    ContactIndicator contact_mode_prev_;
    Eigen::MatrixXd base_contact_Jac_;

    std::string base_link_name  = "Pelvis_Link";
    std::string chest_link_name = "Upperbody_Link";
    std::string lfoot_link_name = "L_Foot_Link";
    std::string rfoot_link_name = "R_Foot_Link";
    std::string lhand_link_name = "L_Wrist2_Link";
    std::string rhand_link_name = "R_Wrist2_Link";
    std::string lshoulder_link_name = "L_Shoulder1_Link";
    std::string rshoulder_link_name = "R_Shoulder1_Link";
    std::string head_link_name  = "Head_Link";
    std::string com_name        = "COM_id";
};