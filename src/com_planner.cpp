#include "com_planner.h"

ComPlanner::ComPlanner() {}

void ComPlanner::planCenterOfMass(const ContactIndicator& contactMode,
                                  const Eigen::Vector3d& com_pos_0,
                                  const Eigen::Vector2d& support_pos_0,
                                  const Eigen::Vector2d& target_step_pos,
                                  const double& current_step_time,
                                  const double& step_duration,
                                  Eigen::Vector2d& com_pos_desired,
                                  Eigen::Vector2d& com_vel_desired,
                                  Eigen::Vector2d& com_acc_desired)
{
    contact_mode_prev_ = contact_mode_;
    contact_mode_ = contactMode;

    if (contact_mode_ != contact_mode_prev_)
    {
        is_com_planning_init = true;
    }

    com_pos_init = com_pos_0.head(2);
    com_vel_init.setZero();
    com_acc_init.setZero();

    constraints_.clear();
    calcCostHessGrad(current_step_time, step_duration, target_step_pos);
    calcEqualityConstraint();
    calcInequalityConstraint(support_pos_0);

    total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();

    if(is_com_planning_init == true)
    {
        total_num_constraints = 0;
        total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();
        for (const auto& c : constraints_) {total_num_constraints += c.A.rows();}

        QP_com_planner.InitializeProblemSize(total_num_state, total_num_constraints);

        A_const   = Eigen::MatrixXd::Zero(total_num_constraints, total_num_state);
        lbA_const = Eigen::VectorXd::Zero(total_num_constraints);
        ubA_const = Eigen::VectorXd::Zero(total_num_constraints);

        is_com_planning_init = false;
    }

    //--- Stack Constraints
    int row_idx  = 0;
    for (const auto& c : constraints_) {
        int rows = c.A.rows();
        A_const.block(row_idx, 0, rows, total_num_state) = c.A;
        lbA_const.segment(row_idx , rows)    = c.lbA;
        ubA_const.segment(row_idx , rows)    = c.ubA;
        row_idx  += rows;
    }

    checkGradHessSize();

    QP_com_planner.EnableEqualityCondition(1e-8);
    QP_com_planner.UpdateMinProblem(Hess, grad);
    QP_com_planner.DeleteSubjectToAx();
    QP_com_planner.UpdateSubjectToAx(A_const, lbA_const, ubA_const);

    bool qp_status = true;
    Eigen::VectorXd X_; X_.setZero(total_num_state);
    if(QP_com_planner.SolveQPoases(500, X_, true))
    {
        coeff_opt = X_.segment(0, coeff_dim);
        qp_status = true;
    }
    else
    {
        //--- CONSTRAINTS VIOLATION CHECKER
        if(is_cannot_solve_qp_init_ == true)   
        {
            Eigen::VectorXd Ax = A_const * X_; 

            for (int i = 0; i < A_const.rows(); ++i)
            {
                double val = Ax(i);
                double l = lbA_const(i);
                double u = ubA_const(i);

                double eps = 1e-5;

                if (val < l - eps)
                {
                    std::cerr << "[Constraint Violation] Row " << i << ": " << val << " < lbA = " << l << std::endl;
                }
                else if (val > u + eps)
                {
                    std::cerr << "[Constraint Violation] Row " << i << ": " << val << " > ubA = " << u << std::endl;
                }
            }
            is_cannot_solve_qp_init_ = false;
        }

        std::cout << "ComPlanner SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
        qp_status = false;
    }

    com_pos_desired = T_pos_current * coeff_opt;
    com_vel_desired = T_vel_current * coeff_opt;
    com_acc_desired = T_acc_current * coeff_opt;

    com_pos_prev = com_pos_desired;
    com_vel_prev = com_vel_desired;
    com_acc_prev = com_acc_desired;
}

void ComPlanner::calcCostHessGrad(const double& current_time, const double& step_duration, const Eigen::Vector2d& target_step_pos)
{
    static bool is_cost_init = true;
    if(is_cost_init == true)
    {
        com_pos_prev = com_pos_init;
        com_vel_prev.setZero();
        com_acc_prev.setZero();
 
        is_cost_init = false;
    }

    //--- Acceleration Energy Minimization
    Eigen::MatrixXd Q_acc_sub; Q_acc_sub.setZero(coeff_dim / 2, coeff_dim/ 2);
    Eigen::MatrixXd Q_acc; Q_acc.setZero(coeff_dim, coeff_dim);
    double rho = 1e-8;

    Q_acc_sub << (400.0 / 7.0) * pow(step_duration, 7), 40.0 * pow(step_duration, 6), 24.0 * pow(step_duration, 5), 10.0 * pow(step_duration, 4), 0.0, 0.0,
                          40.0 * pow(step_duration, 6), 28.8 * pow(step_duration, 5), 18.0 * pow(step_duration, 4),  8.0 * pow(step_duration, 3), 0.0, 0.0,
                          24.0 * pow(step_duration, 5), 18.0 * pow(step_duration, 4), 12.0 * pow(step_duration, 3),  6.0 * pow(step_duration, 2), 0.0, 0.0,
                          10.0 * pow(step_duration, 4),  8.0 * pow(step_duration, 3),  6.0 * pow(step_duration, 2),          4.0 * step_duration, 0.0, 0.0,
                                                   0.0,                          0.0,                          0.0,                          0.0, rho, 0.0,
                                                   0.0,                          0.0,                          0.0,                          0.0, 0.0, rho;

    Q_acc.topLeftCorner(coeff_dim / 2, coeff_dim / 2) = w_acc * Q_acc_sub; 
    Q_acc.bottomRightCorner(coeff_dim / 2, coeff_dim / 2) = w_acc * Q_acc_sub; 

    // //--- Soft Final Constraints
    T_final; calcPositionCoefficientMatrix(T_final, step_duration);
    Eigen::MatrixXd Q_final; Q_final.setZero(coeff_dim, coeff_dim);
    Eigen::VectorXd p_final; p_final.setZero(coeff_dim);

    Q_final = w_final * T_final.transpose() * T_final;
    p_final = (-w_final) * T_final.transpose() * target_step_pos;

    // //--- Derivation from previous solution
    T_pos_current; calcPositionCoefficientMatrix(T_pos_current, current_time);
    T_vel_current; calcVelocityCoefficientMatrix(T_vel_current, current_time);
    T_acc_current; calcAccelerationCoefficientMatrix(T_acc_current, current_time);
    Eigen::MatrixXd Q_prev; Q_prev.setZero(coeff_dim, coeff_dim);
    Eigen::VectorXd p_prev; p_prev.setZero(coeff_dim);

    Q_prev = w_pos_current * T_pos_current.transpose() * T_pos_current 
           + w_vel_current * T_vel_current.transpose() * T_vel_current
           + w_acc_current * T_acc_current.transpose() * T_acc_current;
    p_prev = (-w_pos_current) * T_pos_current.transpose() * com_pos_prev 
           + (-w_vel_current) * T_vel_current.transpose() * com_vel_prev 
           + (-w_acc_current) * T_acc_current.transpose() * com_acc_prev;

    // //--- Path regularization
    Eigen::Vector3d com_x_regularizer_path; com_x_regularizer_path.setZero();
    Eigen::Vector3d com_y_regularizer_path; com_y_regularizer_path.setZero();
    com_x_regularizer_path = DyrosMath::QuinticSpline(current_time, 0.0, step_duration, com_pos_init(0), com_vel_init(0), com_acc_init(0), target_step_pos(0), 0.0, 0.0);
    com_y_regularizer_path = DyrosMath::QuinticSpline(current_time, 0.0, step_duration, com_pos_init(1), com_vel_init(1), com_acc_init(1), target_step_pos(1), 0.0, 0.0);

    Eigen::Vector2d com_pos_regul; com_pos_regul.setZero(); com_pos_regul << com_x_regularizer_path(0), com_y_regularizer_path(0);
    Eigen::Vector2d com_vel_regul; com_vel_regul.setZero(); com_vel_regul << com_x_regularizer_path(1), com_y_regularizer_path(1); 
    Eigen::Vector2d com_acc_regul; com_acc_regul.setZero(); com_acc_regul << com_x_regularizer_path(2), com_y_regularizer_path(2); 

    Eigen::MatrixXd Q_regul; Q_regul.setZero(coeff_dim, coeff_dim);
    Eigen::VectorXd p_regul; p_regul.setZero(coeff_dim);
    Q_regul = w_pos_regul * T_pos_current.transpose() * T_pos_current 
            + w_vel_regul * T_vel_current.transpose() * T_vel_current
            + w_acc_regul * T_acc_current.transpose() * T_acc_current;
    p_regul = (-w_pos_regul) * T_pos_current.transpose() * com_pos_regul 
            + (-w_vel_regul) * T_vel_current.transpose() * com_vel_regul 
            + (-w_acc_regul) * T_acc_current.transpose() * com_acc_regul;

    //--- Total Hessian
    Hess.setZero(coeff_dim, coeff_dim);
    Hess = Q_acc + Q_final + Q_prev + Q_regul;

    //--- Total gradient
    grad.setZero(coeff_dim);
    grad = p_final + p_prev + p_regul;
}

void ComPlanner::calcEqualityConstraint()
{
    //--- Initial Hard Constraints
    Eigen::MatrixXd A_init; A_init.setZero(6, coeff_dim);
    Eigen::VectorXd lbA_init; lbA_init.setZero(6); 
    Eigen::VectorXd ubA_init; ubA_init.setZero(6); 

    T_pos_initial; calcPositionCoefficientMatrix(T_pos_initial, 0.0);
    T_vel_initial; calcVelocityCoefficientMatrix(T_vel_initial, 0.0);
    T_acc_initial; calcAccelerationCoefficientMatrix(T_acc_initial, 0.0);

    A_init.block(0, 0, 2, coeff_dim) = T_pos_initial;
    A_init.block(2, 0, 2, coeff_dim) = T_vel_initial;
    A_init.block(4, 0, 2, coeff_dim) = T_acc_initial;

    lbA_init.segment(0, 2) = com_pos_init;
    lbA_init.segment(2, 2) = com_vel_init;
    lbA_init.segment(4, 2) = com_acc_init;

    ubA_init.segment(0, 2) = com_pos_init;
    ubA_init.segment(2, 2) = com_vel_init;
    ubA_init.segment(4, 2) = com_acc_init;
    
    constraints_.push_back({A_init, lbA_init, ubA_init}); 
}

void ComPlanner::calcInequalityConstraint(const Eigen::Vector2d& support_pos_0)
{
    //--- ZMP Stability Constraints
    Eigen::MatrixXd A_zmp; A_zmp.setZero(2, coeff_dim);
    Eigen::VectorXd lbA_zmp; lbA_zmp.setZero(2); 
    Eigen::VectorXd ubA_zmp; ubA_zmp.setZero(2); 

    A_zmp = T_pos_current - (0.73 / GRAVITY) * T_acc_current;

    // lbA_zmp(0) = support_pos_0(0) - 0.13;
    // lbA_zmp(1) = support_pos_0(1) - 0.08;
    // ubA_zmp(0) = support_pos_0(0) + 0.17;
    // ubA_zmp(1) = support_pos_0(1) + 0.08;
    
    // constraints_.push_back({A_zmp, lbA_zmp, ubA_zmp}); 
}


void ComPlanner::calcPositionCoefficientMatrix(Eigen::MatrixXd& coeff_matrix, const double& time)
{
    Eigen::MatrixXd eta; eta.setZero(1, coeff_dim / 2);
    eta(0, 0) = pow(time, 5);
    eta(0, 1) = pow(time, 4);
    eta(0, 2) = pow(time, 3);
    eta(0, 3) = pow(time, 2);
    eta(0, 4) = time;
    eta(0, 5) = 1.0;


    coeff_matrix.setZero(2, coeff_dim);
    coeff_matrix.topLeftCorner(1, coeff_dim / 2) = eta; 
    coeff_matrix.bottomRightCorner(1, coeff_dim / 2) = eta; 
}

void ComPlanner::calcVelocityCoefficientMatrix(Eigen::MatrixXd& coeff_matrix, const double& time)
{
    Eigen::MatrixXd eta; eta.setZero(1, coeff_dim / 2);
    eta(0, 0) = 5.0 * pow(time, 4);
    eta(0, 1) = 4.0 * pow(time, 3);
    eta(0, 2) = 3.0 * pow(time, 2);
    eta(0, 3) = 2.0 * pow(time, 1);
    eta(0, 4) = 1.0;
    eta(0, 5) = 0.0;


    coeff_matrix.setZero(2, coeff_dim);
    coeff_matrix.topLeftCorner(1, coeff_dim / 2) = eta; 
    coeff_matrix.bottomRightCorner(1, coeff_dim / 2) = eta; 
}

void ComPlanner::calcAccelerationCoefficientMatrix(Eigen::MatrixXd& coeff_matrix, const double& time)
{
    Eigen::MatrixXd eta; eta.setZero(1, coeff_dim / 2);
    eta(0, 0) = 20.0 * pow(time, 3);
    eta(0, 1) = 12.0 * pow(time, 2);
    eta(0, 2) = 6.0  * pow(time, 1);
    eta(0, 3) = 2.0;
    eta(0, 4) = 0.0;
    eta(0, 5) = 0.0;


    coeff_matrix.setZero(2, coeff_dim);
    coeff_matrix.topLeftCorner(1, coeff_dim / 2) = eta; 
    coeff_matrix.bottomRightCorner(1, coeff_dim / 2) = eta; 
}

void ComPlanner::checkGradHessSize()
{
    if(is_gradhess_init_ == true)
    {
        std::cout << "==================================================" << std::endl;
        std::cout << "===== ComPlanner COST & CONSTRAINTS DIM INFO =====" << std::endl;
        std::cout << "==================================================" << std::endl;

        std::cout << "Hess size: " << Hess.rows() << " x " << Hess.cols() << std::endl;
        std::cout << "grad size: " << grad.size() << std::endl;
        std::cout << std::endl;

        std::cout << "A: " << A_const.rows() << " x " << A_const.cols() << std::endl;
        std::cout << "lbA size: " << lbA_const.size() << std::endl;
        std::cout << "ubA size: " << ubA_const.size() << std::endl;
        std::cout << std::endl;
    
        is_gradhess_init_ = false;
    }
}