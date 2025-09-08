// dyn_wbc.h
#ifndef DYN_WBC_H
#define DYN_WBC_H

#include <string>
#include <map>
#include <set>
#include <vector>
#include <Eigen/Dense>
#include <qpOASES.hpp>
// #include <casadi/casadi.hpp>
#include <filesystem>
#include "wholebody_functions.h"
#include "utils.h"
#include "task_definition.h"
#include <iomanip>

class DynWBC
{
public:
    DynWBC(int dof);

    //--- QP WBC 
    CQuadraticProgram QP_Dyn_Wbc;

    bool computeDynamicWBC(Eigen::VectorVQd&qddot_qp, Eigen::VectorXd& contact_wrench);
    void calcCostGrad();
    void calcCostHess();
    void calcEqualityConstraint();
    void calcInequalityConstraint();
    void checkGradHessSize();

    void setRobotSystemParameters(const double& mu_, const double& foot_size_, const double& foot_width_);
    void updateContactState(const ContactIndicator& contactMode);
    void getRobotStates(const Eigen::VectorVQd &q_,
                        const Eigen::VectorVQd &qdot_,
                        const Eigen::VectorVQd &qddot_cmd_,
                        const Eigen::MatrixVVd &Mass_,
                        const Eigen::VectorVQd &Grav_,
                        const Eigen::MatrixXd &base_contact_Jac_);

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
    Eigen::VectorVQd qddot_cmd;
    Eigen::VectorXd qddot_sol; 
    Eigen::VectorXd contact_wrench_sol; 

private:
    int dof;
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

    double W_cwr = 1e-3;
    double W_qddot_b = 1.0;
    double W_energy = 1.0;

    ContactIndicator contact_mode = ContactIndicator::DoubleSupport;
    ContactIndicator contact_mode_prev = ContactIndicator::DoubleSupport;
    int contact_dim = 0;
    int base_dim = 6;
};

#endif  // DYN_WBC_H