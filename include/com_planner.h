#pragma once

#include <eigen3/Eigen/Core>
#include <cmath>
#include <iostream>
#include "wholebody_functions.h"

#include "task_definition.h"

class ComPlanner {
public:
    ComPlanner();
    void planCenterOfMass(const ContactIndicator& contactMode,
                          const Eigen::Vector3d& com_pos_0,
                          const Eigen::Vector2d& support_pos_0,
                          const Eigen::Vector2d& target_step_pos,
                          const double& current_step_time,
                          const double& step_duration,
                          Eigen::Vector2d& com_pos_desired,
                          Eigen::Vector2d& com_vel_desired,
                          Eigen::Vector2d& com_acc_desired);
    void calcCostHessGrad(const double& current_time, const double& step_duration, const Eigen::Vector2d& target_step_pos);
    void calcPositionCoefficientMatrix(Eigen::MatrixXd& coeff_matrix, const double& time);
    void calcVelocityCoefficientMatrix(Eigen::MatrixXd& coeff_matrix, const double& time);
    void calcAccelerationCoefficientMatrix(Eigen::MatrixXd& coeff_matrix, const double& time);
    void calcEqualityConstraint();
    void calcInequalityConstraint(const Eigen::Vector2d& support_pos_0);
    void checkGradHessSize();

    CQuadraticProgram QP_com_planner;
    Eigen::MatrixXd Hess; 
    Eigen::VectorXd grad;
    Eigen::MatrixXd A_const;   
    Eigen::VectorXd lbA_const;
    Eigen::VectorXd ubA_const;
    std::vector<ConstraintMatrix> constraints_;
    int total_num_state = 0;
    int total_num_constraints = 0;

    Eigen::MatrixXd T_pos_current;
    Eigen::MatrixXd T_vel_current;
    Eigen::MatrixXd T_acc_current;

    Eigen::MatrixXd T_final;

    Eigen::MatrixXd T_pos_initial;
    Eigen::MatrixXd T_vel_initial;
    Eigen::MatrixXd T_acc_initial;

    Eigen::Vector2d com_pos_prev;
    Eigen::Vector2d com_vel_prev;
    Eigen::Vector2d com_acc_prev;

    Eigen::VectorXd coeff_opt;

    Eigen::Vector2d com_pos_init;
    Eigen::Vector2d com_vel_init;
    Eigen::Vector2d com_acc_init;


private:
    unsigned int coeff_dim = 12;
    bool is_com_planning_init = true;
    bool is_gradhess_init_ = true;
    bool is_cannot_solve_qp_init_ = true;

    ContactIndicator contact_mode_;
    ContactIndicator contact_mode_prev_;

    double w_acc   = 1e-4;
    double w_final = 1.0; 
    double w_pos_current = 1.0; 
    double w_vel_current = 1.0; 
    double w_acc_current = 1.0; 
    double w_pos_regul = 1.0; 
    double w_vel_regul = 1.0; 
    double w_acc_regul = 1.0; 
};