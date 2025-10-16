#include "kin_wbc.h"

using namespace TOCABI;

KinWBC::KinWBC(RobotData& rd) : rd_(rd) 
{
    task_hierarchy = {
        {{Pelvis, TaskType::Position}, {Pelvis, TaskType::Orientation}},
        {{Left_Foot, TaskType::Position}, {Left_Foot, TaskType::Orientation}, {Right_Foot, TaskType::Position}, {Right_Foot, TaskType::Orientation}},
        {{Head, TaskType::Orientation}},
        {{Left_Hand, TaskType::Position}, {Left_Hand, TaskType::Orientation}, {Right_Hand, TaskType::Position}, {Right_Hand, TaskType::Orientation}}};
}

void KinWBC::computeTaskSpaceKinematicWBC()
{
    //--- Initialization
    qdot_des.setZero();
    Eigen::MatrixVVd Ni = Eigen::MatrixVVd::Identity();

    //--- Nullspace-based Prioritized Task Execution
    for (const auto& task_group : task_hierarchy)
    {
        int m = 3 * task_group.size();
        Eigen::MatrixXd J(m, MODEL_DOF_VIRTUAL);
        Eigen::VectorXd e(m), de(m);

        for (size_t i = 0; i < task_group.size(); ++i)
        {
            const auto& [idx, type] = task_group[i];

            if (type == TaskType::Position)
            {
                J.block(3 * i, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[idx].local_Jac_v;
                Eigen::Vector3d pos_err = rd_.link_[idx].x_traj - rd_.link_[idx].local_xpos;

                de.segment<3>(3 * i) = pos_err;
            }
            else if (type == TaskType::Orientation)
            {
                J.block(3 * i, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[idx].local_Jac_w;
                Eigen::Vector3d ori_err = -DyrosMath::getPhi(rd_.link_[idx].local_rotm, rd_.link_[idx].r_traj);

                de.segment<3>(3 * i) = ori_err;
            }
            else
            {
                ROS_ERROR("Unknown TaskType");
                assert(type == TaskType::Position || type == TaskType::Orientation);
            }
        }

        Eigen::MatrixXd J_pre = J * Ni;
        Eigen::MatrixXd J_pinv = DyrosMath::pinv_SVD(J_pre);

        qdot_des += J_pinv * (de - J * qdot_des);
        Ni *= (Eigen::MatrixVVd::Identity() - J_pinv * J_pre);
    }

    qdot_des = safetyFilter();

    rd_.q_dot_desired_virtual = qdot_des;
    rd_.q_dot_desired = rd_.q_dot_desired_virtual.tail(MODEL_DOF);

    rd_.q_desired_virtual = rd_.local_q_virtual_.head(MODEL_DOF_VIRTUAL) + rd_.q_dot_desired_virtual;
    rd_.q_desired = rd_.q_desired_virtual.tail(MODEL_DOF);
}

Eigen::VectorVQd KinWBC::safetyFilter()
{
    constraints_.clear();
    calcCostHess();
    calcCostGrad();
    calcEqualityConstraint();
    calcInequalityConstraint();

    total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();

    static bool is_filter_init_ = true;
    if(is_filter_init_ == true)
    {
        total_num_constraints = 0;
        total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();
        for (const auto& c : constraints_) {total_num_constraints += c.A.rows();}

        QP_safety_filter.InitializeProblemSize(total_num_state, total_num_constraints);

        A_const   = Eigen::MatrixXd::Zero(total_num_constraints, total_num_state);
        lbA_const = Eigen::VectorXd::Zero(total_num_constraints);
        ubA_const = Eigen::VectorXd::Zero(total_num_constraints);

        qdot_safety.setZero();

        is_filter_init_ = false;
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

    QP_safety_filter.EnableEqualityCondition(1e-8);
    QP_safety_filter.UpdateMinProblem(Hess, grad);
    QP_safety_filter.DeleteSubjectToAx();
    QP_safety_filter.UpdateSubjectToAx(A_const, lbA_const, ubA_const);

    bool qp_status = true;
    Eigen::VectorXd X_; X_.setZero(total_num_state);
    if(QP_safety_filter.SolveQPoases(500, X_, true))
    {
        qdot_safety = X_.segment(0, MODEL_DOF_VIRTUAL);
        qp_status = true;
    }
    else
    {
        //--- CONSTRAINTS VIOLATION CHECKER
        if(is_cannot_solve_qp_ == true)   
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
            is_cannot_solve_qp_ = false;
        }

        qdot_safety.setZero();
        std::cout << "Kin WBC SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
        qp_status = false;
    }

    return (qdot_safety);
}

void KinWBC::calcCostHess()
{
    Hess.setIdentity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
}

void KinWBC::calcCostGrad()
{
    grad.setZero(MODEL_DOF_VIRTUAL);
    grad = (-1.0) * Hess * qdot_des;
}

void KinWBC::calcEqualityConstraint()
{
}

void KinWBC::calcInequalityConstraint()
{
    //--- (1) Joint position constraints
    Eigen::MatrixXd A_qpos; A_qpos.setZero(MODEL_DOF, MODEL_DOF_VIRTUAL);
    A_qpos.rightCols(MODEL_DOF).setIdentity();

    double alpha_qpos = 1.0;
    double eps_qpos = 50.0;
    Eigen::VectorXd lbA_qpos; lbA_qpos.setZero(MODEL_DOF); 
    Eigen::VectorXd ubA_qpos; ubA_qpos.setZero(MODEL_DOF); 
    for(int i = 0; i < MODEL_DOF; i++)
    {
        lbA_qpos(i) = min(max(alpha_qpos * (rd_.q_pos_l_lim(i) - rd_.q_(i)) + (1.0 / eps_qpos), rd_.q_vel_l_lim(i)), rd_.q_vel_h_lim(i));
        ubA_qpos(i) = max(min(alpha_qpos * (rd_.q_pos_h_lim(i) - rd_.q_(i)) - (1.0 / eps_qpos), rd_.q_vel_h_lim(i)), rd_.q_vel_l_lim(i));
    }
    
    constraints_.push_back({A_qpos, lbA_qpos, ubA_qpos}); 

    //--- (2) Reachability constraints
    // const int m = static_cast<int>(grad_reachability_.size()); 
    // double alpha_reachability = 1.0;
    // double eps_reachability = 50.0;
    // Eigen::MatrixXd A_reachability; A_reachability.setZero(m, MODEL_DOF_VIRTUAL);
    // Eigen::VectorXd lbA_reachability; lbA_reachability.setZero(m);

    // for (int i = 0; i < m; ++i) {
    //         A_reachability.block(i, 0, 1, MODEL_DOF_VIRTUAL) = grad_reachability_[i];
    //         lbA_reachability(i) = (-1.0) * alpha_reachability * cbf_reachability_[i] + (1.0 / eps_reachability) * grad_reachability_[i].squaredNorm();
    // }

    // constraints_.push_back({   
    //     A_reachability,
    //     lbA_reachability,
    //     Eigen::VectorXd::Constant(A_reachability.rows(), std::numeric_limits<double>::infinity())
    // });

    // ---self, environment, reachability, ... + alpha (singularity)
} 

void KinWBC::getReachabilityConstraints(const std::vector<Eigen::MatrixXd> &J_reachability_, const std::vector<double> &h_reachability_)
{
    // const int m = static_cast<int>(J_reachability_.size()); 
    // if (m == 0) return;
    // assert(m == static_cast<int>(h_reachability_.size()) && "Reachability Constraints's Hessian and gradients size mismatch");

    // const int n = static_cast<int>(J_reachability_[0].cols());
    // for (int i = 0; i < m; ++i) {
    //     assert(J_reachability_[i].rows() == 1 && J_reachability_[i].cols() == n && "J_i must be 1 x n");
    // }

    // static bool is_reach_init_ = true;
    // if (is_reach_init_ == true) 
    // {
    //     grad_reachability_.assign(m, Eigen::MatrixXd::Zero(1, n));
    //     cbf_reachability_.assign(m, 0.0); 

    //     is_reach_init_ = false;
    // }

    // for (int i = 0; i < m; ++i) {
    //     grad_reachability_[i] = J_reachability_[i];
    //     cbf_reachability_[i]  = h_reachability_[i];
    // }
}

void KinWBC::checkGradHessSize()
{
    static bool is_gradhess_init_ = true;
    if(is_gradhess_init_ == true)
    {
        std::cout << "==============================================" << std::endl;
        std::cout << "===== KinWBC COST & CONSTRAINTS DIM INFO =====" << std::endl;
        std::cout << "==============================================" << std::endl;

        std::cout << "total_num_state: " << total_num_state << std::endl;
        std::cout << "total_num_constraints: " << total_num_constraints << std::endl;
        std::cout << std::endl;

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