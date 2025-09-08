// dyn_wbc.cpp
#include "dyn_wbc.h"
#include <iostream>

ofstream dataWBC1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC1.txt");
ofstream dataWBC2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC2.txt");
ofstream dataWBC6("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC6.txt");

ofstream qpHessGrad("/home/kwan/catkin_ws/src/tocabi_cc/data/qpHessGrad.txt");

using namespace Eigen;
using namespace qpOASES;

DynWBC::DynWBC(int dof_) : dof(dof_) { }

bool DynWBC::computeDynamicWBC(Eigen::VectorVQd&qddot_qp, Eigen::VectorXd& contact_wrench_qp)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    constraints_.clear();
    calcCostHess();
    calcCostGrad();
    calcEqualityConstraint();
    calcInequalityConstraint();
    if (contact_mode != contact_mode_prev)
    {
        is_wbc_init_ = true;
        is_gradhess_init_ = true;
        std::cout << "!!!!!!!!!!CONTACT TRIGGER!!!!!!!!!!";
        std::cout << "Transition from [" << contactIndicatorToString(contact_mode_prev)
                << "] to [" << contactIndicatorToString(contact_mode) << "]" << std::endl;
    }

    total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();

    if(is_wbc_init_ == true)
    {
        total_num_constraints = 0;
        for (const auto& c : constraints_) {total_num_constraints += c.A.rows();}

        //--- Initialization
        QP_Dyn_Wbc.InitializeProblemSize(total_num_state, total_num_constraints);

        A_const   = Eigen::MatrixXd::Zero(total_num_constraints, total_num_state);
        lbA_const = Eigen::VectorXd::Zero(total_num_constraints);
        ubA_const = Eigen::VectorXd::Zero(total_num_constraints);

        std::cout << "total_num_state: " << total_num_state << std::endl;
        std::cout << "total_num_constraints: " << total_num_constraints << std::endl;

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
        contact_wrench_sol  = X_.segment(0, contact_dim);
        qddot_sol = X_.segment(contact_dim, dof);
        real_t score = QP_Dyn_Wbc.returnObjVal();
        // std::cout << "##### Contact Wrench QP cost value: " << score << std::endl;
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

                double eps = 0.0;

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

    //---Return 
    qddot_qp = qddot_sol;
    contact_wrench_qp = contact_wrench_sol;

    return(qp_status);
}

void DynWBC::setRobotSystemParameters(const double& mu_, const double& foot_size_, const double& foot_width_)
{
    //--- Friction, Contact, Torque limit constraints
    mu = mu_;
    foot_size = foot_size_; 
    foot_width = foot_width_; 
}

void DynWBC::updateContactState(const ContactIndicator& contactMode)
{
    contact_mode_prev = contact_mode;
    contact_mode = contactMode;

    if(contact_mode == ContactIndicator::DoubleSupport)
    {
        contact_dim = 12;
    }
    else if (contact_mode == ContactIndicator::LeftSingleSupport || contact_mode == ContactIndicator::RightSingleSupport)
    {
        contact_dim = 6;
    }

    contact_wrench_cmd.setZero(contact_dim);
}

void DynWBC::getRobotStates(const Eigen::VectorVQd &q_,
                            const Eigen::VectorVQd &qdot_,
                            const Eigen::VectorVQd &qddot_cmd_,
                            const Eigen::MatrixVQVQd &Mass_,
                            const Eigen::VectorVQd &Grav_,
                            const Eigen::MatrixXd &base_contact_Jac_) 
{
    //--- Robot States
    q = q_;
    qdot = qdot_;
    qddot_cmd = qddot_cmd_;
    M = Mass_;
    G = Grav_;

    base_contact_Jac.setZero(base_contact_Jac_.rows(), base_contact_Jac_.cols());
    base_contact_Jac = base_contact_Jac_;

    base_contact_Jac_T.setZero(base_contact_Jac_.cols(), base_contact_Jac_.rows());
    base_contact_Jac_T = base_contact_Jac.transpose();

    Sa_T.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF); Sa_T.bottomRows(MODEL_DOF).setIdentity();
    Sa.setZero(MODEL_DOF, MODEL_DOF_VIRTUAL); Sa = Sa_T.transpose();
    Sf.setZero(base_dim, MODEL_DOF_VIRTUAL); Sf.leftCols(base_dim).setIdentity();

    //--- Friction cone constraints (https://scaron.info/robotics/wrench-friction-cones.html)
    Eigen::MatrixXd U_fric_dsp; U_fric_dsp.setZero(34, 12);
    Eigen::MatrixXd U_fric_ssp; U_fric_ssp.setZero(17, 6);
    double X = foot_size  / 2.0; 
    double Y = foot_width / 2.0; 
    U_fric_ssp <<  0,  0,            -1,   0,   0,  0,
                  -1,  0,           -mu,   0,   0,  0,
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
    U_fric_dsp.topLeftCorner(17, 6) = U_fric_ssp;
    U_fric_dsp.bottomRightCorner(17, 6) = U_fric_ssp;

    if(contact_mode == ContactIndicator::DoubleSupport)
    {
        A_fric.setZero(34, contact_dim + dof);
        lbA_fric.setZero(34);
        ubA_fric.setZero(34);

        A_fric.leftCols(contact_dim) = U_fric_dsp;
    }
    else if(contact_mode == ContactIndicator::LeftSingleSupport || contact_mode == ContactIndicator::RightSingleSupport)
    {
        A_fric.setZero(17, contact_dim + dof);
        lbA_fric.setZero(17);
        ubA_fric.setZero(17);

        A_fric.leftCols(contact_dim) = U_fric_ssp;
    }
}

void DynWBC::calcCostHess()
{
    Hess.setZero(contact_dim + dof, contact_dim + dof);
    Hess.topLeftCorner(contact_dim, contact_dim) = W_cwr * Eigen::MatrixXd::Identity(contact_dim, contact_dim);
    Hess.bottomRightCorner(dof, dof) = W_qddot_b * Eigen::MatrixXd::Identity(dof, dof);
    Hess.bottomRightCorner(dof, dof) = W_energy * M;
}

void DynWBC::calcCostGrad()
{
    grad.setZero(contact_dim + dof);
    grad.head(contact_dim) -= W_cwr * contact_wrench_cmd;
    grad.tail(dof) -= W_qddot_b * qddot_cmd;
}

void DynWBC::calcEqualityConstraint()
{
    //--- (1) Floating base dynamics
    Eigen::MatrixXd A_fl; A_fl.setZero(base_dim, contact_dim + dof);
    Eigen::VectorXd lbA_fl; lbA_fl.setZero(base_dim);
    Eigen::VectorXd ubA_fl; ubA_fl.setZero(base_dim);

    A_fl.leftCols(contact_dim) = Sf * base_contact_Jac_T;
    A_fl.rightCols(dof) = -Sf * M;
    lbA_fl = Sf * G;
    ubA_fl = Sf * G;
    constraints_.push_back({A_fl, lbA_fl, ubA_fl});
}

void DynWBC::calcInequalityConstraint()
{
    //--- (2) Friction cone constraints
    constraints_.push_back({   
        A_fric,
        Eigen::VectorXd::Constant(A_fric.rows(), -std::numeric_limits<double>::infinity()),
        ubA_fric
    });
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
