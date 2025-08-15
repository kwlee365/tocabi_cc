// dyn_wbc.cpp
#include "dyn_wbc.h"
#include <iostream>

ofstream dataWBC1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC1.txt");
ofstream dataWBC2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC2.txt");
ofstream dataWBC3("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC3.txt");
ofstream dataWBC4("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC4.txt");
ofstream dataWBC5("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC5.txt");
ofstream dataWBC6("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC6.txt");

ofstream qpHessGrad("/home/kwan/catkin_ws/src/tocabi_cc/data/qpHessGrad.txt");


using namespace Eigen;
using namespace qpOASES;

DynWBC::DynWBC(int dof) : dof_(dof) { }

bool DynWBC::computeDynamicWBC(const std::vector<std::vector<TaskInfo>>& wbd_dynamic_task, Eigen::VectorQd& torque_unbound)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //--- Initialization
    constraints_.clear();
    calcCostHess(wbd_dynamic_task);
    calcCostGrad(wbd_dynamic_task);
    calcEqualityConstraint();
    calcInequalityConstraint();

    if (contact_mode_local != contact_mode_local_prev)
    {
        is_wbc_init_ = true;
        is_gradhess_init_ = true;
        std::cout << "!!!!!!!!!!CONTACT TRIGGER!!!!!!!!!!";
        std::cout << "Transition from [" << contactIndicatorToString(contact_mode_local_prev)
                << "] to [" << contactIndicatorToString(contact_mode_local) << "]" << std::endl;
    }

    if(is_wbc_init_ == true)
    {
        total_num_constraints = 0;
        total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();
        for (const auto& c : constraints_) {total_num_constraints += c.A.rows();}

        //--- Initialization
        QP_Dyn_Wbc.InitializeProblemSize(total_num_state, total_num_constraints);

        A_const   = Eigen::MatrixXd::Zero(total_num_constraints, total_num_state);
        lbA_const = Eigen::VectorXd::Zero(total_num_constraints);
        ubA_const = Eigen::VectorXd::Zero(total_num_constraints);

        torque_sol.setZero();

        is_wbc_init_ = false;
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

    QP_Dyn_Wbc.EnableEqualityCondition(1e-8);
    QP_Dyn_Wbc.UpdateMinProblem(Hess, grad);
    QP_Dyn_Wbc.DeleteSubjectToAx();
    QP_Dyn_Wbc.UpdateSubjectToAx(A_const, lbA_const, ubA_const);

    bool qp_status = true;
    Eigen::VectorXd X_; X_.setZero(total_num_state);
    if(QP_Dyn_Wbc.SolveQPoases(500, X_, true))
    {
        torque_sol  = X_.segment(0, MODEL_DOF);

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


        std::cout << "Dyn WBC SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
        qp_status = false;
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    dataWBC6 << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;

    dataWBC1 << torque_sol.transpose() << std::endl;

    //---Return 
    torque_unbound = torque_sol;

    return(qp_status);
}

void DynWBC::setRobotSystemParameters(const double& mu_, const double& foot_size_, const double& foot_width_, const double& force_z_max_, const double& force_z_min_, 
                                      const Eigen::VectorQd& torque_lim_, const Eigen::VectorQd& q_pos_l_lim_, const Eigen::VectorQd& q_pos_h_lim_, const Eigen::VectorQd& q_vel_l_lim_, const Eigen::VectorQd& q_vel_h_lim_)
{
    //--- Friction, Contact, Torque limit constraints
    mu = mu_;
    foot_size = foot_size_; 
    foot_width = foot_width_; 
    force_z_max = force_z_max_;
    force_z_min = force_z_min_;

    torque_lim = torque_lim_;
    q_pos_l_lim = q_pos_l_lim_;
    q_pos_h_lim = q_pos_h_lim_;
    q_vel_l_lim = q_vel_l_lim_;
    q_vel_h_lim = q_vel_h_lim_;
}

void DynWBC::setWbcWeights(const Eigen::VectorQd& W_torque_,
                           const Eigen::VectorQd& W_energy_, 
                           const Eigen::VectorXd& W_contact_)
 {
    W_torque = W_torque_;

    W_energy = W_energy_;

    W_contact.setZero(W_contact_.size());
    W_contact = W_contact_;

    //--- Print
    static bool weight_checker = true;
    if(weight_checker == true)
    {
        std::cout << "\n====== W_torque ======" << std::endl;
        std::cout << W_torque.transpose() << std::endl;

        std::cout << "\n====== W_energy ======" << std::endl;
        std::cout << W_energy.transpose() << std::endl;

        std::cout << "\n====== W_contact ======" << std::endl;
        std::cout << W_contact.transpose() << std::endl;

        weight_checker = false;
    }
}

Eigen::VectorQd DynWBC::computeNominalTorque()
{
    torque_nominal.setZero();

    torque_nominal = J_task_T * F_task + N_task * torque_impedance;

    return (torque_nominal);
}
                            
void DynWBC::computeContactWrench(const ContactIndicator& contactMode, const double& MG_)
{
    contact_mode_local_prev = contact_mode_local;
    contact_mode_local = contactMode;

    MG = MG_;

    if(contact_mode_local == ContactIndicator::DoubleSupport)
    {
        F_contact.setZero(12);
        F_gravity.setZero(12);
        J_contact_inv_T.setZero(12, MODEL_DOF);

        F_gravity(2) = MG / 2.0;
        F_gravity(8) = MG / 2.0;
    }
    else if (contact_mode_local == ContactIndicator::LeftSingleSupport || contact_mode_local == ContactIndicator::RightSingleSupport)
    {
        F_contact.setZero(6);
        F_gravity.setZero(6);
        J_contact_inv_T.setZero(6, MODEL_DOF);

        F_gravity(2) = MG;
    }
}

void DynWBC::getRobotStates(const std::vector<std::vector<TaskInfo>> &wbd_dynamic_task,
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
                            const Eigen::VectorQd &torque_impedance_) 
{
    //--- Robot States
    static bool is_state_init = true;
    q_prev = q;

    if(is_state_init == true)
    {
        q_prev.setZero();
        q_prev = q_;

        is_state_init = false;
        std::cout << "getRobotStates()" << std::endl;
    }

    q = q_;
    qdot = qdot_;

    //--- Contact Consistent Whole-body Dynamics
    M.setZero(); G.setZero(); A.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF);
    M = Mass_;
    M_inv = Mass_inv_;
    G = base_contact_N_ * Grav_ + base_contact_Jac_.transpose() * base_contact_lambda_ * base_contact_Jac_dot_ * qdot_;
    A = base_contact_N_.rightCols(MODEL_DOF);

    std::set<std::string> visited;

    //--- Dynamically consistent inverse
    int rows = lambda_task_.rows();
    int cols = lambda_task_.cols();
    lambda_task = Eigen::MatrixXd::Zero(rows, cols);
    lambda_task = lambda_task_;

    rows = J_task_T_.rows();
    cols = J_task_T_.cols();
    J_task_T = Eigen::MatrixXd::Zero(rows, cols);
    J_task_T = J_task_T_;

    rows = N_task_.rows();
    cols = N_task_.cols();
    N_task = Eigen::MatrixXd::Zero(rows, cols);
    N_task = N_task_;

    int size = F_task_.size();
    F_task = Eigen::VectorXd::Zero(size);
    F_task = F_task_;

    torque_impedance.setZero();
    torque_impedance = torque_impedance_;

    //--- || qddot - qddot_nom ||^2
    J_torque_nominal.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF);
    J_torque_nominal = (M_inv * base_contact_N_).rightCols(MODEL_DOF);

    //--- Relationship btw Contact wrench and Toruqe
    J_contact_inv_T = (-1.0) * base_contact_Jac_inv_T_.rightCols(MODEL_DOF);
    F_contact = F_gravity + (base_contact_lambda_ * base_contact_Jac_dot_ * qdot_) - base_contact_Jac_inv_T_ * Grav_;

    //--- Friction cone constraints (https://scaron.info/robotics/wrench-friction-cones.html)
    Eigen::MatrixXd U_fric_dsp; U_fric_dsp.setZero(32, 12);
    Eigen::MatrixXd U_fric_ssp; U_fric_ssp.setZero(16, 6);
    double X = foot_size  / 2.0; 
    double Y = foot_width / 2.0; 
    U_fric_ssp << -1,  0,           -mu,   0,   0,  0,
                  +1,  0,           -mu,   0,   0,  0,
                   0, -1,           -mu,   0,   0,  0,
                   0, +1,           -mu,   0,   0,  0,
                   0,  0,            -Y,  -1,   0,  0,
                   0,  0,            -Y,  +1,   0,  0,
                   0,  0,            -X,   0,  -1,  0,
                   0,  0,            -X,   0,  +1,  0,
                  -Y, -X, -(X + Y) * mu, -mu, +mu, -1,
                  +Y, +X, -(X + Y) * mu, +mu, -mu, -1,
                  +Y, -X, -(X + Y) * mu, +mu, +mu, -1,
                  +Y, +X, -(X + Y) * mu, +mu, +mu, -1,
                  +Y, -X, -(X + Y) * mu, +mu, +mu, +1,
                  +Y, +X, -(X + Y) * mu, +mu, -mu, +1,
                  -Y, -X, -(X + Y) * mu, -mu, -mu, +1,
                  -Y, +X, -(X + Y) * mu, -mu, +mu, +1;
    U_fric_dsp.topLeftCorner(16, 6) = U_fric_ssp;
    U_fric_dsp.bottomRightCorner(16, 6) = U_fric_ssp;

    if(contact_mode_local == ContactIndicator::DoubleSupport)
    {
        A_fric.setZero(32, MODEL_DOF);
        ubA_fric.setZero(32);

        A_fric = (-1.0) * U_fric_dsp * (base_contact_Jac_inv_T_.rightCols(MODEL_DOF));
        ubA_fric = U_fric_dsp * (base_contact_lambda_ * base_contact_Jac_dot_ * qdot_ - base_contact_Jac_inv_T_ * Grav_);
    }
    else if(contact_mode_local == ContactIndicator::LeftSingleSupport || contact_mode_local == ContactIndicator::RightSingleSupport)
    {
        A_fric.setZero(16, MODEL_DOF);
        ubA_fric.setZero(16);

        A_fric = (-1.0) * U_fric_ssp * (base_contact_Jac_inv_T_.rightCols(MODEL_DOF));
        ubA_fric = U_fric_ssp * (base_contact_lambda_ * base_contact_Jac_dot_ * qdot_ - base_contact_Jac_inv_T_ * Grav_);
    }
}

void DynWBC::calcCostHess(const std::vector<std::vector<TaskInfo>>& wbd_dynamic_task)
{
    Hess.setZero(MODEL_DOF, MODEL_DOF);

    Hess += W_torque.asDiagonal(); 

    Hess += J_contact_inv_T.transpose() * W_contact.asDiagonal() * J_contact_inv_T; 

    Hess += W_energy.asDiagonal() * (A.transpose() * M_inv.transpose() * A);
}

void DynWBC::calcCostGrad(const std::vector<std::vector<TaskInfo>>& wbd_dynamic_task)
{
    grad.setZero(MODEL_DOF);

    grad -= W_torque.asDiagonal() * torque_impedance;

    grad -= J_contact_inv_T.transpose() * W_contact.asDiagonal() * F_contact;

    grad -= W_energy.asDiagonal() * (A.transpose() * M_inv.transpose() * G);
}

void DynWBC::calcEqualityConstraint()
{

}

void DynWBC::calcInequalityConstraint()
{
    // //--- (1) Torque constraints
    // Eigen::MatrixQQd A_torque; A_torque.setIdentity();
    // Eigen::VectorQd lbA_torque; lbA_torque = (-1.0) * torque_lim;
    // Eigen::VectorQd ubA_torque; ubA_torque = (+1.0) * torque_lim;
    // constraints_.push_back({A_torque, lbA_torque, ubA_torque});   // --- Torque Boundary (Size 33)

    //--- (2) Joint position constraints
    // Eigen::MatrixQQd A_qpos; A_qpos = (M_inv * A).bottomRows(MODEL_DOF);
    // Eigen::VectorQd lbA_qpos; lbA_qpos = alpha1 * alpha2 * (q_pos_l_lim - q.tail(MODEL_DOF)) - (alpha1 + alpha2) * qdot.tail(MODEL_DOF) + (M_inv * G).tail(MODEL_DOF);
    // Eigen::VectorQd ubA_qpos; ubA_qpos = alpha1 * alpha2 * (q_pos_h_lim - q.tail(MODEL_DOF)) - (alpha1 + alpha2) * qdot.tail(MODEL_DOF) + (M_inv * G).tail(MODEL_DOF);
    // constraints_.push_back({A_qpos, lbA_qpos, ubA_qpos}); 

    Eigen::MatrixQQd A_qpos;   A_qpos.setZero();
    Eigen::VectorQd  lbA_qpos; lbA_qpos.setZero();
    Eigen::VectorQd  ubA_qpos; ubA_qpos.setZero();
    Eigen::VectorQd  qdot_a; qdot_a = qdot.tail(MODEL_DOF);
    Eigen::VectorQd  q_a; q_a = q.tail(MODEL_DOF);
    double qdot_norm = qdot.norm();
    double cu  = 20.0;    // larger than upperbound of Gravity vector
    double alpha = 100.0;
    double alpha_e = 1000.0;

    A_qpos = (qdot.tail(MODEL_DOF).transpose() / alpha_e).replicate(MODEL_DOF, 1);
    for (int i = 0; i < MODEL_DOF; i++)
    {
        // ubA_qpos(i) = qdot.transpose() * G + alpha_e * qdot_a(i) + alpha * (-qdot.transpose() * M * qdot + alpha_e * (q_a(i) - q_pos_l_lim(i)));
        ubA_qpos(i) = + (qdot_a(i) + alpha * (q_a(i) - q_pos_l_lim(i))) - (cu  / alpha_e) * qdot_norm *(1.0 + alpha * qdot_norm); 
    }
    constraints_.push_back({
        A_qpos, 
        Eigen::VectorXd::Constant(A_qpos.rows(), -std::numeric_limits<double>::infinity()),
        ubA_qpos}); 

    A_qpos = (qdot.tail(MODEL_DOF).transpose() / alpha_e).replicate(MODEL_DOF, 1);
    for (int i = 0; i < MODEL_DOF; i++)
    {
        // ubA_qpos(i) = qdot.transpose() * G - alpha_e * qdot_a(i) + alpha * (-qdot.transpose() * M * qdot + alpha_e * (q_pos_h_lim(i) - q_a(i)));
        ubA_qpos(i) = - (qdot_a(i) + alpha * (q_a(i) - q_pos_h_lim(i))) - (cu  / alpha_e) * qdot_norm *(1.0 + alpha * qdot_norm) ; 
    }
    constraints_.push_back({
        A_qpos, 
        Eigen::VectorXd::Constant(A_qpos.rows(), -std::numeric_limits<double>::infinity()),
        ubA_qpos}); 

        
    //--- (3) Friction cone constraints
    constraints_.push_back({   
        A_fric,
        Eigen::VectorXd::Constant(A_fric.rows(), -std::numeric_limits<double>::infinity()),
        ubA_fric
    });

    //--- (4) Reachability constraints
    const int m = static_cast<int>(Hess_reachability_.size()); 
    A_reachability.setZero(m, MODEL_DOF);
    lbA_reachability.setZero(m);

    for (int i = 0; i < m; ++i) {
            A_reachability.block(i, 0, 1, MODEL_DOF) = grad_reachability_[i] * M_inv * A;
            lbA_reachability(i) = 
                                + (-1.0) * ( qdot.transpose() * Hess_reachability_[i] * qdot)(0) 
                                + (-1.0) * ((alpha3 + alpha4) * grad_reachability_[i] * qdot)(0) 
                                + (-1.0) * ( alpha3 * alpha4 * cbf_reachability_[i]);
                                + (+1.0) * (grad_reachability_[i] * M_inv * G)(0);
    }

    // const int m = static_cast<int>(Hess_reachability_.size()); 
    // A_reachability.setZero(m, MODEL_DOF);
    // ubA_reachability.setZero(m);

    // A_reachability = (qdot.tail(MODEL_DOF).transpose() / alpha_e).replicate(m, 1);
    // for (int i = 0; i < m; ++i) {
    //         // ubA_qpos(i) = - cu * qdot_norm *(1.0 + alpha * qdot_norm) + alpha_e * ((grad_reachability_[i] * qdot)(0) + alpha * cbf_reachability_[i]); 
    //     ubA_reachability(i) =  ((grad_reachability_[i] * qdot)(0) + alpha * cbf_reachability_[i]) - (cu  / alpha_e) * qdot_norm *(1.0 + alpha * qdot_norm) ; 
    // }

    constraints_.push_back({   
        A_reachability,
        Eigen::VectorXd::Constant(A_reachability.rows(), -std::numeric_limits<double>::infinity()),
        ubA_reachability
    });

}

void DynWBC::getReachabilityConstraints(const std::vector<Eigen::MatrixXd> &J_reachability_, const std::vector<double> &h_reachability_)
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
        Hess_reachability_.assign(m, Eigen::MatrixXd::Identity(n, n));
        Hess_reachability_prev_.assign(m, Eigen::MatrixXd::Identity(n, n));
        grad_reachability_.assign(m, Eigen::MatrixXd::Zero(1, n));
        grad_reachability_prev_.assign(m, Eigen::MatrixXd::Zero(1, n));
        cbf_reachability_.assign(m, 0.0); 

        for (int i = 0; i < m; ++i) {
            grad_reachability_prev_[i] = J_reachability_[i];
        }

        is_reach_init_ = false;
    }

    Eigen::VectorXd s; s.setZero(n);
    s = q - q_prev;

    for (int i = 0; i < m; ++i) {

        grad_reachability_[i] = J_reachability_[i];
        cbf_reachability_[i]  = h_reachability_[i];

        // BFGS Update
        Eigen::MatrixXd y; y.setZero(1, n);
        y = grad_reachability_[i] - grad_reachability_prev_[i];

        double rho = 1.0 / (y * s)(0);

        Hess_reachability_[i] = Hess_reachability_prev_[i] 
                              + rho * (y.transpose() * y) 
                              - (1.0 / (s.transpose() * Hess_reachability_prev_[i] * s)(0)) * (Hess_reachability_prev_[i] * s * s.transpose() * Hess_reachability_prev_[i]);

        Hess_reachability_prev_[i] = (y * s)(0) / (y * y.transpose())(0) * Eigen::MatrixXd::Identity(n, n);

        // Previous gradient update 
        grad_reachability_prev_[i] = grad_reachability_[i];
    }

}

void DynWBC::checkGradHessSize()
{
    if(is_gradhess_init_ == true)
    {
        std::cout << "==============================================" << std::endl;
        std::cout << "===== DynWBC COST & CONSTRAINTS DIM INFO =====" << std::endl;
        std::cout << "==============================================" << std::endl;

        std::cout << "Hess size: " << Hess.rows() << " x " << Hess.cols() << std::endl;
        std::cout << "grad size: " << grad.size() << std::endl;
        std::cout << std::endl;

        std::cout << "A: " << A_const.rows() << " x " << A_const.cols() << std::endl;
        std::cout << "lbA size: " << lbA_const.size() << std::endl;
        std::cout << "ubA size: " << ubA_const.size() << std::endl;
        std::cout << std::endl;

        qpHessGrad << "A_const: " << std::endl;
        qpHessGrad << A_const << std::endl;
        qpHessGrad << " " << std::endl;
        qpHessGrad << "lbA_const: " << std::endl;
        qpHessGrad << lbA_const.transpose() << std::endl;
        qpHessGrad << " " << std::endl;
        qpHessGrad << "ubA_const:  " << std::endl;
        qpHessGrad << ubA_const.transpose() << std::endl;
        qpHessGrad << " " << std::endl;

        is_gradhess_init_ = false;
    }
}
