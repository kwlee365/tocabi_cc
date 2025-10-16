#pragma once

#include <eigen3/Eigen/Core>
#include <map>
#include <vector>
#include <string>
#include "wholebody_functions.h"
#include "utils.h"

class KinWBC {
public:
    KinWBC(RobotData& rd);

    void computeTaskSpaceKinematicWBC();

private:
    RobotData &rd_;
    std::vector<std::vector<TaskInfo>> task_hierarchy;

    bool is_cannot_solve_qp_ = true;

    Eigen::VectorVQd safetyFilter();
    CQuadraticProgram QP_safety_filter;

        void calcCostHess();
        void calcCostGrad();
        void calcEqualityConstraint();
        void calcInequalityConstraint();
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

    Eigen::VectorVQd qdot_des;
    Eigen::VectorVQd qdot_safety;
};