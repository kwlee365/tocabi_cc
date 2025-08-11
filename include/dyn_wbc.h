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

struct ConstraintMatrix {
    Eigen::MatrixXd A;
    Eigen::VectorXd lbA;
    Eigen::VectorXd ubA;
};

class DynWBC
{
public:
    DynWBC(int dof);

    //--- QP WBC 
    CQuadraticProgram QP_Dyn_Wbc;

    bool computeDynamicWBC(const std::vector<std::vector<TaskInfo>>& wbd_dynamic_task, Eigen::VectorQd& torque_unbound);
    void calcCostGrad(const std::vector<std::vector<TaskInfo>>& wbd_dynamic_task);
    void calcCostHess(const std::vector<std::vector<TaskInfo>>& wbd_dynamic_task);
    void calcEqualityConstraint();
    void calcInequalityConstraint();
    void checkGradHessSize();

    void setRobotSystemParameters(const double& mu_, const double& foot_size_, const double& foot_width_, const double& force_z_max_, const double& force_z_min_, 
                                  const Eigen::VectorQd& torque_lim_, const Eigen::VectorQd& q_pos_l_lim_, const Eigen::VectorQd& q_pos_h_lim_, const Eigen::VectorQd& q_vel_l_lim_, const Eigen::VectorQd& q_vel_h_lim_);
    
    void setWbcWeights(const Eigen::VectorQd& W_torque_,
                       const Eigen::VectorQd& W_energy_, 
                       const Eigen::VectorXd& W_contact_);
                        
    void computeContactWrench(const ContactIndicator& contactMode,const double& MG);

    Eigen::VectorQd computeNominalTorque();

    void getRobotStates(const std::vector<std::vector<TaskInfo>> &wbd_dynamic_task,
                        const Eigen::VectorVQd &q_,
                        const Eigen::VectorVQd &qdot_,
                        const Eigen::MatrixVVd &Mass_,
                        const Eigen::MatrixVVd &Mass_inv_,
                        const Eigen::VectorVQd &Grav_,
                        const Eigen::MatrixXd &base_contact_Jac_,
                        const Eigen::MatrixXd &base_contact_Jac_dot_,
                        const Eigen::MatrixXd &base_contact_lambda_,
                        const Eigen::MatrixXd &base_contact_Jac_inv_T_,
                        const Eigen::MatrixVVd &base_contact_N_,
                        const Eigen::MatrixXd &lambda_task_,
                        const Eigen::MatrixXd &J_task_T_,
                        const Eigen::MatrixXd &N_task_,
                        const Eigen::VectorXd &F_task_,
                        const Eigen::VectorQd &torque_impedance_);

    Eigen::MatrixXd Hess;  // HESSIAN
    Eigen::VectorXd grad;  // GRADIENT
    Eigen::MatrixXd A_const;    
    Eigen::VectorXd lbA_const;  
    Eigen::VectorXd ubA_const;  
    std::vector<ConstraintMatrix> constraints_; // CONSTRAINTS
    int total_num_state = 0;
    int total_num_constraints = 0;

    Eigen::VectorQd torque_sol; 

private:
    int dof_;
    bool is_gradhess_init_ = true;
    bool is_wbc_init_ = true;
    bool is_cannot_solve_qp_init_ = true;

    double mu = 0.0;
    double foot_size = 0.0; 
    double foot_width = 0.0; 
    double force_z_max = 0.0;
    double force_z_min = 0.0;
    double MG = 0.0;

    double alpha1 = 100.0;
    double alpha2 = 100.0;
    double alpha3 = 10.0;

    //--- Local eigen variables
    Eigen::MatrixVVd M; 
    Eigen::MatrixVVd M_inv; 
    Eigen::VectorVQd G; 
    Eigen::MatrixXd A; 
    Eigen::VectorVQd q;
    Eigen::VectorVQd qdot;

    Eigen::VectorQd torque_lim;
    Eigen::VectorQd q_pos_l_lim;
    Eigen::VectorQd q_pos_h_lim;
    Eigen::VectorQd q_vel_l_lim;
    Eigen::VectorQd q_vel_h_lim;

    Eigen::VectorQd torque_prev;
    Eigen::MatrixXd lambda_task;
    Eigen::MatrixXd J_task_T;
    Eigen::MatrixXd N_task;
    Eigen::VectorXd F_task; 
    Eigen::VectorQd torque_impedance;
    Eigen::MatrixXd J_torque_nominal;
    Eigen::MatrixXd J_contact_inv_T;
    Eigen::VectorXd F_contact; 
    Eigen::VectorXd F_gravity; 
    Eigen::MatrixXd J_fric;
    Eigen::VectorXd ubA_fric;
    Eigen::VectorQd torque_nominal;

    Eigen::VectorQd W_torque;
    Eigen::VectorQd W_energy;
    Eigen::VectorXd W_contact;

    ContactIndicator contact_mode_local = ContactIndicator::DoubleSupport;
    ContactIndicator contact_mode_local_prev = ContactIndicator::DoubleSupport;
};

#endif  // DYN_WBC_H