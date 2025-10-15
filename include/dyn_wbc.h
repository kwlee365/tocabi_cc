// dyn_wbc.h
#ifndef DYN_WBC_H
#define DYN_WBC_H

#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "wholebody_functions.h"
#include "utils.h"
#include <iomanip>

class DynWBC
{
public:
    DynWBC(RobotData& rd);

    //--- QP WBC 
    CQuadraticProgram QP_Dyn_Wbc;

    void computeDynamicWBC();
    void calcDesiredJointAcceleration();
    void computeTotalTorqueCommand();

    void calcCostGrad();
    void calcCostHess();
    void calcEqualityConstraint();
    void calcInequalityConstraint();
    void checkGradHessSize();

    void setRobotSystemParameters(const double& mu_, const double& foot_size_, const double& foot_width_);
    void updateContactState();
    void updateRobotStates();

    Eigen::MatrixXd Hess;  // HESSIAN
    Eigen::VectorXd grad;  // GRADIENT
    Eigen::MatrixXd A_const;    
    Eigen::VectorXd lbA_const;  
    Eigen::VectorXd ubA_const;  
    std::vector<ConstraintMatrix> constraints_; // CONSTRAINTS
    int total_num_state = 0;
    int total_num_constraints = 0;

    Eigen::Vector6d base_impedance_cmd;
    Eigen::VectorXd contact_wrench_cmd;
    Eigen::Vector6d qddot_b_cmd;
    Eigen::VectorQd qddot_a_cmd;
    Eigen::VectorVQd qdot_des;
    Eigen::VectorVQd qddot_cmd;
    Eigen::VectorXd qddot_qp; 
    Eigen::VectorXd contact_wrench_qp; 

private:
    RobotData &rd_;

    bool is_gradhess_init_ = true;
    bool is_wbc_init_ = true;
    bool is_cannot_solve_qp_init_ = true;

    double mu = 0.0;
    double foot_size = 0.0; 
    double foot_width = 0.0; 
    double MG = 0.0;

    //--- Local eigen variables
    Eigen::MatrixVVd M; 
    Eigen::VectorVQd G; 
    Eigen::MatrixXd base_contact_Jac;
    Eigen::MatrixXd base_contact_Jac_T;
    Eigen::MatrixXd Sa_T;
    Eigen::MatrixXd Sa;  
    Eigen::MatrixXd Sf;  

    Eigen::VectorVQd q;
    Eigen::VectorVQd qdot;

    Eigen::MatrixXd A_fric;
    Eigen::VectorXd lbA_fric;
    Eigen::VectorXd ubA_fric;

    double W_cwr = 1e-5;
    double W_qddot = 1.0;
    double W_energy = 0.5;

    int contact_dim = 12;
    int contact_dim_prev = 12;
    int base_dim = 6;
};

#endif  // DYN_WBC_H