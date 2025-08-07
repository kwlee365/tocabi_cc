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

bool DynWBC::computeDynamicWBC(const std::vector<std::vector<TaskInfo>>& task_hierarchy_, Eigen::VectorQd& torque_unbound)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //--- Initialization
    constraints_.clear();
    calcCostHess(task_hierarchy_);
    calcCostGrad(task_hierarchy_);
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
    dataWBC2 << (J_task_inv_T["L_Wrist2_Link"] * torque_sol).transpose() << std::endl; 
    dataWBC3 << F_task["L_Wrist2_Link"].transpose() << std::endl; 
    dataWBC4 << (J_contact_inv_T * torque_sol).transpose() << std::endl; 
    dataWBC5 << F_contact.transpose() << std::endl; 

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

void DynWBC::setWbcWeights(const std::vector<std::vector<TaskInfo>>& task_hierarchy_,
                           const std::map<std::string, Eigen::VectorXd>& W_task_, const Eigen::VectorQd& W_energy_, const Eigen::VectorXd& W_contact_, const Eigen::VectorQd& W_torque_prev_)
{
    for (const auto& task_group : task_hierarchy_)
    {
        int m = 3 * task_group.size();
        for (const auto& [name, type] : task_group){ W_task[name] = Eigen::VectorXd::Zero(m);}

        std::set<std::string> visited;
        for (size_t i = 0; i < task_group.size(); ++i)
        {
            const auto& [name, type] = task_group[i];

            if (visited.count(name))
                continue;
            visited.insert(name);

            W_task[name] = W_task_.at(name);
        }
    }

    W_energy = W_energy_;

    W_contact.setZero(W_contact_.size());
    W_contact = W_contact_;

    W_torque_prev = W_torque_prev_;
}

void DynWBC::computeTaskImpedance(const std::vector<std::vector<TaskInfo>>& task_hierarchy_,
                                  const std::map<std::string, Eigen::Vector3d>& task_Kp, const std::map<std::string, Eigen::Vector3d>& task_Kv, 
                                  const std::map<std::string, Eigen::Vector3d>& x_desired, const std::map<std::string, Eigen::Vector3d>& dx_desired, const std::map<std::string, Eigen::Vector3d>& ddx_desired,
                                  const std::map<std::string, Eigen::Matrix3d>& R_desired, const std::map<std::string, Eigen::Vector3d>& w_desired, const std::map<std::string, Eigen::Vector3d>& dw_desired,
                                  const std::map<std::string, Eigen::Vector3d>& base_ee_pos, const std::map<std::string, Eigen::Matrix3d>& base_ee_rot,
                                  const std::map<std::string, Eigen::Vector3d>& base_ee_v, const std::map<std::string, Eigen::Vector3d>& base_ee_w)
{
    for (const auto& task_group : task_hierarchy_)
    {
        int m = 3 * task_group.size();
        for (const auto& [name, type] : task_group){ F_task[name] = Eigen::VectorXd::Zero(m);}

        for (size_t i = 0; i < task_group.size(); ++i)
        {
            const auto& [name, type] = task_group[i];

            if (type == TaskType::Position)
            {
                Eigen::Vector3d Kp_vec = task_Kp.at(name); 
                Eigen::Vector3d Kv_vec = task_Kv.at(name);

                Eigen::Vector3d pos_err = x_desired.at(name)  - base_ee_pos.at(name);
                Eigen::Vector3d vel_err = dx_desired.at(name) - base_ee_v.at(name); 

                F_task.at(name).segment<3>(3 * i) = ddx_desired.at(name) + Kp_vec.asDiagonal() * pos_err + Kv_vec.asDiagonal() * vel_err;
            }
            else if (type == TaskType::Orientation)
            {
                Eigen::Vector3d Kp_vec = task_Kp.at(name);
                Eigen::Vector3d Kv_vec = task_Kv.at(name);
                
                Eigen::Vector3d ori_err = -DyrosMath::getPhi(base_ee_rot.at(name), R_desired.at(name));
                // Eigen::Vector3d ori_err = -getOrientationError(base_ee_rot.at(name), R_desired.at(name));
                Eigen::Vector3d vel_err = (w_desired.at(name) - base_ee_w.at(name)); 

                F_task.at(name).segment<3>(3 * i) = dw_desired.at(name) + Kp_vec.asDiagonal() * ori_err + Kv_vec.asDiagonal() * vel_err;
            }
            else
            {
                ROS_ERROR("Unknown TaskType for link [%s], type value: %d",
                        name.c_str(), static_cast<int>(type));
                assert(false && "Unknown TaskType");
            }
        }
    }
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

void DynWBC::getRobotStates(const std::vector<std::vector<TaskInfo>>& task_hierarchy_,
                            const Eigen::VectorVQd& q_,
                            const Eigen::VectorVQd& qdot_,
                            const Eigen::MatrixVVd& Mass_, 
                            const Eigen::MatrixVVd& Mass_inv_, 
                            const Eigen::VectorVQd& Grav_, 
                            const Eigen::MatrixXd& base_contact_Jac_,
                            const Eigen::MatrixXd& base_contact_Jac_dot_,
                            const Eigen::MatrixXd& base_contact_lambda_,
                            const Eigen::MatrixXd& base_contact_Jac_inv_T_,
                            const Eigen::MatrixVVd& base_contact_N_, 
                            const std::map<std::string, Eigen::MatrixXd>& base_task_Jac_inv_T_S_T_,
                            const Eigen::VectorQd& torque_prev_)
{
    //--- Robot States
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
    for (const auto& task_group : task_hierarchy_)
    {
        for (const auto& [name, type] : task_group)
        {
            if (visited.count(name))
                continue;
            visited.insert(name);

            int rows = base_task_Jac_inv_T_S_T_.at(name).rows();
            int cols = base_task_Jac_inv_T_S_T_.at(name).cols();
            J_task_inv_T[name] = Eigen::MatrixXd::Zero(rows, cols);
            J_task_inv_T[name] = base_task_Jac_inv_T_S_T_.at(name);
        }
    }

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
        J_fric.setZero(32, MODEL_DOF);
        ubA_fric.setZero(32);

        J_fric = (-1.0) * U_fric_dsp * (base_contact_Jac_inv_T_.rightCols(MODEL_DOF));
        ubA_fric = U_fric_dsp * (base_contact_lambda_ * base_contact_Jac_dot_ * qdot_ - base_contact_Jac_inv_T_ * Grav_);
    }
    else if(contact_mode_local == ContactIndicator::LeftSingleSupport || contact_mode_local == ContactIndicator::RightSingleSupport)
    {
        J_fric.setZero(16, MODEL_DOF);
        ubA_fric.setZero(16);

        J_fric = (-1.0) * U_fric_ssp * (base_contact_Jac_inv_T_.rightCols(MODEL_DOF));
        ubA_fric = U_fric_ssp * (base_contact_lambda_ * base_contact_Jac_dot_ * qdot_ - base_contact_Jac_inv_T_ * Grav_);
    }

    //--- Torque command regularization
    torque_prev = torque_prev_;
}

void DynWBC::calcCostHess(const std::vector<std::vector<TaskInfo>>& task_hierarchy_)
{
    Hess.setZero(MODEL_DOF, MODEL_DOF);

    std::set<std::string> visited;
    for (const auto& task_group : task_hierarchy_)
    {
        for (const auto& [name, type] : task_group)
        { 
            if (visited.count(name))
                continue;
            visited.insert(name);

            Hess += J_task_inv_T[name].transpose() * W_task[name].asDiagonal() * J_task_inv_T[name]; 
        }
    }

    Hess += J_contact_inv_T.transpose() * W_contact.asDiagonal() * J_contact_inv_T; 

    Hess += W_energy.asDiagonal() * (A.transpose() * M_inv.transpose() * A);

    Hess += W_torque_prev.asDiagonal();
}

void DynWBC::calcCostGrad(const std::vector<std::vector<TaskInfo>>& task_hierarchy_)
{
    grad.setZero(MODEL_DOF);

    std::set<std::string> visited;
    for (const auto& task_group : task_hierarchy_)
    {
        for (const auto& [name, type] : task_group)
        { 
            if (visited.count(name))
                continue;
            visited.insert(name);

            grad -= J_task_inv_T[name].transpose() * W_task[name].asDiagonal() * F_task[name]; 
        }
    }

    grad -= J_contact_inv_T.transpose() * W_contact.asDiagonal() * F_contact;

    grad -= W_energy.asDiagonal() * (A.transpose() * M_inv.transpose() * G);

    grad -= W_torque_prev.asDiagonal() * torque_prev;
}

void DynWBC::calcEqualityConstraint()
{

}

void DynWBC::calcInequalityConstraint()
{
    // //--- (1) Torque constraints
    Eigen::MatrixQQd A_torque; A_torque.setIdentity();
    Eigen::VectorQd lbA_torque; lbA_torque = (-1.0) * torque_lim;
    Eigen::VectorQd ubA_torque; ubA_torque = (+1.0) * torque_lim;
    constraints_.push_back({A_torque, lbA_torque, ubA_torque});   // --- Torque Boundary (Size 33)

    // //--- (2) Joint position constraints
    Eigen::MatrixQQd A_qpos; A_qpos = (M_inv * A).bottomRows(MODEL_DOF);
    Eigen::VectorQd lbA_qpos; lbA_qpos = alpha1 * alpha2 * (q_pos_l_lim - q.tail(MODEL_DOF)) - (alpha1 + alpha2) * qdot.tail(MODEL_DOF) + (M_inv * G).tail(MODEL_DOF);
    Eigen::VectorQd ubA_qpos; ubA_qpos = alpha1 * alpha2 * (q_pos_h_lim - q.tail(MODEL_DOF)) - (alpha1 + alpha2) * qdot.tail(MODEL_DOF) + (M_inv * G).tail(MODEL_DOF);
    constraints_.push_back({A_qpos, lbA_qpos, ubA_qpos}); 

    // //--- (3) Friction cone constraints
    constraints_.push_back({   
        J_fric,
        Eigen::VectorXd::Constant(J_fric.rows(), -std::numeric_limits<double>::infinity()),
        ubA_fric
    });

    // constraints_.push_back({   
    //     A_LF_fric_min,
    //     Eigen::VectorXd::Constant(A_LF_fric_min.rows(), -std::numeric_limits<double>::infinity()),
    //     Eigen::VectorXd::Zero(A_LF_fric_min.rows())
    // });

    // constraints_.push_back({   
    //     A_RF_fric_max,
    //     Eigen::VectorXd::Constant(A_RF_fric_max.rows(), -std::numeric_limits<double>::infinity()),
    //     Eigen::VectorXd::Zero(A_RF_fric_max.rows())
    // });

    // constraints_.push_back({    
    //     A_RF_fric_min,
    //     Eigen::VectorXd::Constant(A_RF_fric_min.rows(), -std::numeric_limits<double>::infinity()),
    //     Eigen::VectorXd::Zero(A_RF_fric_min.rows())
    // });
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
