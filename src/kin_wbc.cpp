#include "kin_wbc.h"

KinWBC::KinWBC(int dof) : dof_(dof) {}

void KinWBC::computeTaskSpaceKinematicWBC(
    const std::vector<std::vector<TaskInfo>>& task_hierarchy,
    const ContactIndicator& contactMode,
    const std::map<std::string, Eigen::Vector3d>& x_desired, const std::map<std::string, Eigen::Vector3d>& dx_desired, const std::map<std::string, Eigen::Vector3d>& ddx_desired,
    const std::map<std::string, Eigen::Matrix3d>& R_desired, const std::map<std::string, Eigen::Vector3d>& w_desired, const std::map<std::string, Eigen::Vector3d>& dw_desired,
    const std::map<std::string, Eigen::Vector3d>& task_pos_Kp, const std::map<std::string, Eigen::Vector3d>& task_ori_Kp, 
    const std::map<std::string, Eigen::Vector3d>& base_ee_pos, const std::map<std::string, Eigen::Matrix3d>& base_ee_rot,
    const std::map<std::string, Eigen::Vector3d>& base_ee_v, const std::map<std::string, Eigen::Vector3d>& base_ee_w, 
    const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_v, const std::map<std::string, Eigen::Matrix3Vd>& base_Jac_w, const Eigen::Matrix6Vd& base_CMM, 
    const Eigen::VectorVQd& qdot, Eigen::VectorVQd& qdot_des)
{
    //--- Initialization
    qdot_des = Eigen::VectorVQd::Zero();
    Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(dof_, dof_);
    contact_mode_prev_ = contact_mode_;
    contact_mode_ = contactMode;

    //--- Nullspace-based Prioritized Task Execution
    int row_offset = 0;
    for (const auto& task_group : task_hierarchy)
    {
        int m = 3 * task_group.size();
        Eigen::MatrixXd J(m, dof_);
        Eigen::MatrixXd Jdot(m, dof_);
        Eigen::VectorXd e(m), de(m), dde(m);

        for (size_t i = 0; i < task_group.size(); ++i)
        {
            const auto& [name, type] = task_group[i];
            if (type == TaskType::Position)
            {
                Eigen::Vector3d Kp_vec = task_pos_Kp.at(name); 

                J.block(3 * i, 0, 3, dof_) = base_Jac_v.at(name);
                Eigen::Vector3d pos_err = x_desired.at(name)  - base_ee_pos.at(name);
                Eigen::Vector3d vel_err = dx_desired.at(name) - base_ee_v.at(name); 

                e.segment<3>(3 * i)  = pos_err;
                de.segment<3>(3 * i) = dx_desired.at(name) + Kp_vec.asDiagonal() * pos_err;
                dde.segment<3>(3 * i) = ddx_desired.at(name);
            }
            else if (type == TaskType::Orientation)
            {
                Eigen::Vector3d Kp_vec = task_ori_Kp.at(name);
                
                J.block(3 * i, 0, 3, dof_) = base_Jac_w.at(name);
                Eigen::Vector3d ori_err = -DyrosMath::getPhi(base_ee_rot.at(name), R_desired.at(name));
                Eigen::Vector3d vel_err = (w_desired.at(name) - base_ee_w.at(name)); 

                e.segment<3>(3 * i)   = ori_err;
                de.segment<3>(3 * i)  = w_desired.at(name) + Kp_vec.asDiagonal() * ori_err;
                dde.segment<3>(3 * i) = dw_desired.at(name);
            }
            else
            {
                ROS_ERROR("Unknown TaskType");
                assert(type == TaskType::Position || type == TaskType::Orientation);
            }
        }

        row_offset += m;  

        Eigen::MatrixXd J_pre = J * Ni;
        Eigen::MatrixXd J_pinv = DyrosMath::pinv_SVD(J_pre);

        qdot_des += J_pinv * (de  - J * qdot_des);
        Ni *= (Eigen::MatrixXd::Identity(dof_, dof_) - J_pinv * J_pre);
    }

    // Eigen::MatrixXd CMM_yaw = base_CMM.bottomRows(1);
    // Eigen::MatrixXd CMM_yaw_pre = CMM_yaw * Ni;
    // Eigen::MatrixXd CMM_yaw_pinv = DyrosMath::pinv_SVD(CMM_yaw_pre);

    // qdot_des += CMM_yaw_pinv * (- CMM_yaw * qdot_des);
    // Ni *= (Eigen::MatrixXd::Identity(dof_, dof_) - CMM_yaw_pinv * CMM_yaw_pre);
}

void KinWBC::safetyFilter(Eigen::VectorVQd& qdot_des, const Eigen::VectorVQd& q,
                          const Eigen::VectorQd& q_pos_l_lim, const Eigen::VectorQd& q_pos_h_lim,
                          const Eigen::VectorQd& q_vel_l_lim, const Eigen::VectorQd& q_vel_h_lim)
{
    constraints_.clear();
    calcCostHess();
    calcCostGrad(qdot_des);
    calcEqualityConstraint();
    calcInequalityConstraint(q, q_pos_l_lim, q_pos_h_lim, q_vel_l_lim, q_vel_h_lim);
    if (contact_mode_ != contact_mode_prev_)
    {
        if(contact_mode_prev_ == ContactIndicator::DoubleSupport)
        {
            is_filter_init_ = true;
            is_gradhess_init_ = true;
            std::cout << "!!!!!!!!!!CONTACT TRIGGER!!!!!!!!!!";
            std::cout << "Transition from [" << contactIndicatorToString(contact_mode_)
                      << "] to [" << contactIndicatorToString(contact_mode_) << "]" << std::endl;
        }
    }

    total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();

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
        qdot_safety = X_.segment(0, dof_);
        // real_t score = QP_safety_filter.returnObjVal();
        // std::cout << "##### Safety Filter QP cost value: " << score << std::endl;
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


        std::cout << "Kin WBC SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
        qp_status = false;
    }

    qdot_des = qdot_safety;
}

void KinWBC::calcCostHess()
{
    Hess.setIdentity(dof_, dof_);
}

void KinWBC::calcCostGrad(const Eigen::VectorVQd& qdot_des)
{
    grad.setZero(dof_);
    grad = (-1.0) * Hess * qdot_des;
}

void KinWBC::calcEqualityConstraint()
{
}

void KinWBC::calcInequalityConstraint(const Eigen::VectorVQd& q_, const Eigen::VectorQd& q_pos_l_lim_, const Eigen::VectorQd& q_pos_h_lim_, const Eigen::VectorQd& q_vel_l_lim_, const Eigen::VectorQd& q_vel_h_lim_)
{
    //--- (1) Joint position constraints
    Eigen::MatrixXd A_qpos; A_qpos.setZero(MODEL_DOF, dof_);
    A_qpos.rightCols(MODEL_DOF).setIdentity();

    double alpha_qpos = 1.0;
    double eps_qpos = 50.0;
    Eigen::VectorXd lbA_qpos; lbA_qpos.setZero(MODEL_DOF); 
    Eigen::VectorXd ubA_qpos; ubA_qpos.setZero(MODEL_DOF); 
    Eigen::VectorQd q_a; q_a.setZero(MODEL_DOF);
    q_a = q_.tail(MODEL_DOF);
    for(int i = 0; i < MODEL_DOF; i++)
    {
        lbA_qpos(i) = min(max(alpha_qpos * (q_pos_l_lim_(i) - q_a(i)) + (1.0 / eps_qpos), q_vel_l_lim_(i)), q_vel_h_lim_(i));
        ubA_qpos(i) = max(min(alpha_qpos * (q_pos_h_lim_(i) - q_a(i)) - (1.0 / eps_qpos), q_vel_h_lim_(i)), q_vel_l_lim_(i));
    }
    
    constraints_.push_back({A_qpos, lbA_qpos, ubA_qpos}); 

    //--- (2) Reachability constraints
    const int m = static_cast<int>(grad_reachability_.size()); 
    double alpha_reachability = 1.0;
    double eps_reachability = 50.0;
    Eigen::MatrixXd A_reachability; A_reachability.setZero(m, dof_);
    Eigen::VectorXd lbA_reachability; lbA_reachability.setZero(m);

    for (int i = 0; i < m; ++i) {
            A_reachability.block(i, 0, 1, dof_) = grad_reachability_[i];
            lbA_reachability(i) = (-1.0) * alpha_reachability * cbf_reachability_[i] + (1.0 / eps_reachability) * grad_reachability_[i].squaredNorm();
    }

    // constraints_.push_back({   
    //     A_reachability,
    //     lbA_reachability,
    //     Eigen::VectorXd::Constant(A_reachability.rows(), std::numeric_limits<double>::infinity())
    // });
}

void KinWBC::getReachabilityConstraints(const std::vector<Eigen::MatrixXd> &J_reachability_, const std::vector<double> &h_reachability_)
{
    const int m = static_cast<int>(J_reachability_.size()); 
    if (m == 0) return;
    assert(m == static_cast<int>(h_reachability_.size()) && "Reachability Constraints's Hessian and gradients size mismatch");

    const int n = static_cast<int>(J_reachability_[0].cols());
    for (int i = 0; i < m; ++i) {
        assert(J_reachability_[i].rows() == 1 && J_reachability_[i].cols() == n && "J_i must be 1 x n");
    }

    static bool is_reach_init_ = true;
    if (is_reach_init_ == true) 
    {
        grad_reachability_.assign(m, Eigen::MatrixXd::Zero(1, n));
        cbf_reachability_.assign(m, 0.0); 

        is_reach_init_ = false;
    }

    for (int i = 0; i < m; ++i) {
        grad_reachability_[i] = J_reachability_[i];
        cbf_reachability_[i]  = h_reachability_[i];
    }
}

void KinWBC::checkGradHessSize()
{
    if(is_gradhess_init_ == true)
    {
        std::cout << "==============================================" << std::endl;
        std::cout << "===== KinWBC COST & CONSTRAINTS DIM INFO =====" << std::endl;
        std::cout << "==============================================" << std::endl;

        std::cout << "Hess size: " << Hess.rows() << " x " << Hess.cols() << std::endl;
        std::cout << "grad size: " << grad.size() << std::endl;
        std::cout << std::endl;

        std::cout << "A: " << A_const.rows() << " x " << A_const.cols() << std::endl;
        std::cout << "lbA size: " << lbA_const.size() << std::endl;
        std::cout << "ubA size: " << ubA_const.size() << std::endl;
        std::cout << std::endl;
    
        // std::cout << "Hess: " << std::endl;
        // std::cout << Hess << std::endl;
        // std::cout << "grad: " << std::endl;
        // std::cout << grad << std::endl;
        // std::cout << std::endl;

        // std::cout << "A_const: " << std::endl;
        // std::cout << A_const << std::endl;
        // std::cout << " " << std::endl;
        // std::cout << "lbA_const: " << std::endl;
        // std::cout << lbA_const.transpose() << std::endl;
        // std::cout << " " << std::endl;
        // std::cout << "ubA_const:  " << std::endl;
        // std::cout << ubA_const.transpose() << std::endl;
        // std::cout << " " << std::endl;

        is_gradhess_init_ = false;
    }
}