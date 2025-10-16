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

    void computeDynamicWBC();
    void computeTotalTorqueCommand();

    //--- Setter
    void setFrictionCoefficient(const double& mu_);
    void setFootDimension(const double& foot_size_, const double& foot_width_);
    void setJointTrackingWeight(const double& W_qddot_);
    void setContactWrenchRegularizationWeight(const double& W_cwr_);
    void setAccelEnergyMinimizationWeight(const double& W_energy_);


private:
    RobotData &rd_;

    CQuadraticProgram QP_Dyn_Wbc;
        void calcCostGrad();
        void calcCostHess();
        void calcEqualityConstraint();
        void calcInequalityConstraint();
        void checkGradHessSize();

        void updateContactState();
        void updateRobotStates();
        void calcDesiredJointAcceleration();

        Eigen::MatrixXd Hess;  // HESSIAN
        Eigen::VectorXd grad;  // GRADIENT
        Eigen::MatrixXd A_const;    
        Eigen::VectorXd lbA_const;  
        Eigen::VectorXd ubA_const;  
        std::vector<ConstraintMatrix> constraints_; // CONSTRAINTS
        int total_num_state = 0;
        int total_num_constraints = 0;

    Eigen::VectorXd contact_wrench_cmd;
    Eigen::VectorVQd qddot_cmd;
    Eigen::VectorXd contact_wrench_qp; 
    Eigen::VectorXd qddot_qp; 

    bool is_gradhess_init_ = true;
    bool is_wbc_init_ = true;
    bool is_cannot_solve_qp_ = true;

    double mu = 0.0;
    double foot_size = 0.0; 
    double foot_width = 0.0; 

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

    double W_qddot = 1.0;
    double W_cwr = 1e-5;
    double W_energy = 1.0;

    int contact_dim = 12;
    int contact_dim_prev = 12;
};

#endif  // DYN_WBC_H