// dyn_wbc.h
#ifndef DYN_WBC_H
#define DYN_WBC_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <casadi/casadi.hpp>
#include <filesystem>
#include "wholebody_functions.h"
#include "task_definition.h"

struct ConstraintMatrix {
    Eigen::MatrixXd A;
    Eigen::VectorXd lbA;
    Eigen::VectorXd ubA;
};

class DynWBC
{
public:
    DynWBC(int dof);

    //--- Library Path
    std::string current_path = std::filesystem::current_path().parent_path().string();
    std::string library_path = current_path + "/catkin_ws/src/tocabi_cc/wbc_lib/";
    std::string library_name = "lib_wbc_func.so";

    //--- QP WBC 
    void setRobotSystemParameters(const double& mu, const double& foot_size, const double& foot_width, const int& contact_dim, 
                                  const Eigen::VectorQd& torque_lim, const Eigen::VectorQd& q_pos_l_lim, const Eigen::VectorQd& q_pos_h_lim, 
                                  const double& force_z_max, const double& force_z_min);
    void setWbcWeights(const Eigen::VectorVQd& W_Q, const Eigen::VectorQd& W_torque, const Eigen::VectorXd& W_lambda, const Eigen::VectorQd& W_energy);
    void getRobotStates(const Eigen::MatrixVQVQd& H, 
                        const Eigen::VectorVQd& G, 
                        const Eigen::MatrixXd& J_c, 
                        const Eigen::VectorVQd& qddot_des_from_ik,
                        const Eigen::VectorQd& q,
                        const Eigen::VectorQd& qdot);

    void calcCostGrad();
    void calcCostHess();
    void calcEqualityConstraint();
    void calcInequalityConstraint();

    Eigen::VectorQd computeDynamicWBC();
    void checkGradHessSize();
    void JointLimitChecker();

    CQuadraticProgram QP_Dyn_Wbc;
    Eigen::MatrixXd Hess_;  // HESSIAN
    Eigen::VectorXd grad_;  // GRADIENT
    Eigen::MatrixXd A_;    
    Eigen::VectorXd lbA_;  
    Eigen::VectorXd ubA_;  
    std::vector<ConstraintMatrix> constraints_; // CONSTRAINTS
    int total_num_state = 0;
    int total_num_constraints = 0;

    //--- CasADi Utils
    void casadiFunctionCall();

    template <typename EigenType>
    void EigenToCasadiDM(casadi::DM &casadi_dm, const EigenType &eigen_data, int rows, int cols);

    template <typename ReturnType>
    ReturnType CasadiDMVectorToEigen(const std::vector<casadi::DM> &casadi_dm_vector);

private:
    int dof_;
    bool is_gradhess_init_ = true;
    bool is_wbc_init_ = true;

    casadi::Function J_v_func_, J_vv_func_;
    casadi::Function ceq0_func_, ceq0_v_func_;
    casadi::Function ceq1_func_, ceq1_v_func_;

    casadi::Function cineq1_max_func_, cineq1_max_v_func_;
    casadi::Function cineq2_max_func_, cineq2_max_v_func_;
    casadi::Function cineq3_max_func_, cineq3_max_v_func_;
    casadi::Function cineq4_max_func_, cineq4_max_v_func_;
    casadi::Function cineq5_max_func_, cineq5_max_v_func_;
    casadi::Function cineq6_max_func_, cineq6_max_v_func_;
    casadi::Function cineq7_max_func_, cineq7_max_v_func_;

    casadi::Function cineq1_min_func_, cineq1_min_v_func_;
    casadi::Function cineq2_min_func_, cineq2_min_v_func_;
    casadi::Function cineq3_min_func_, cineq3_min_v_func_;
    casadi::Function cineq4_min_func_, cineq4_min_v_func_;
    casadi::Function cineq5_min_func_, cineq5_min_v_func_;
    casadi::Function cineq6_min_func_, cineq6_min_v_func_;
    casadi::Function cineq7_min_func_, cineq7_min_v_func_;

    //--- Local variables
    casadi::DM H_; 
    casadi::DM G_; 
    casadi::DM J_c_; 
    casadi::DM qddot_des_from_ik_; 
    casadi::DM torque_; 
    casadi::DM torque_sol_; 
    casadi::DM lambda_; 
    casadi::DM torque_lim_; 
    casadi::DM W_Q_; 
    casadi::DM W_torque_; 
    casadi::DM W_lambda_;
    casadi::DM W_torque_prev_;
    casadi::DM q_pos_l_lim_;
    casadi::DM q_pos_h_lim_;
    casadi::DM q_;
    casadi::DM qdot_;
    //--- Solution
    Eigen::VectorQd torque_sol;
    Eigen::Vector12d contact_wrench_sol;

    double mu_ = 0.0;
    double foot_size_ = 0.0; 
    double foot_width_ = 0.0; 
    int contact_dim_ = 0.0;
    double force_z_max_ = 0.0;
    double force_z_min_ = 0.0;

    double alpha1 = 10.0;
    double alpha2 = 10.0;
};

#endif  // DYN_WBC_H