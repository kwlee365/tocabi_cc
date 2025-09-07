// dyn_wbc.cpp
#include "dyn_wbc.h"
#include <iostream>

ofstream dataWBC1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC1.txt");
ofstream dataWBC2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC2.txt");
ofstream dataWBC6("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC6.txt");

ofstream qpHessGrad("/home/kwan/catkin_ws/src/tocabi_cc/data/qpHessGrad.txt");

using namespace Eigen;
using namespace qpOASES;

DynWBC::DynWBC(int dof) : dof_(dof) { }

bool DynWBC::computeDynamicWBC(Eigen::VectorXd& contact_wrench)
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
    contact_wrench = contact_wrench_sol;

    return(qp_status);
}

void DynWBC::setRobotSystemParameters(const double& mu_, const double& foot_size_, const double& foot_width_, const Eigen::VectorQd& torque_lim_)
{
    //--- Friction, Contact, Torque limit constraints
    mu = mu_;
    foot_size = foot_size_; 
    foot_width = foot_width_; 

    torque_lim = torque_lim_;
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

    S_T.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF); S_T.bottomRows(MODEL_DOF).setIdentity();
    S.setZero(MODEL_DOF, MODEL_DOF_VIRTUAL);   S = S_T.transpose();

    base_impedance_cmd.setZero();
    base_impedance_cmd = (M * qddot_cmd + G).topRows(6);

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

    if(contact_mode == ContactIndicator::DoubleSupport)
    {
        A_fric.setZero(32, contact_dim);
        lbA_fric.setZero(32);
        ubA_fric.setZero(32);

        A_fric = U_fric_dsp;
    }
    else if(contact_mode == ContactIndicator::LeftSingleSupport || contact_mode == ContactIndicator::RightSingleSupport)
    {
        A_fric.setZero(16, contact_dim);
        lbA_fric.setZero(16);
        ubA_fric.setZero(16);

        A_fric = U_fric_ssp;
    }
}

void DynWBC::calcCostHess()
{
    Hess.setZero(contact_dim, contact_dim);
    Hess = base_contact_Jac * base_contact_Jac_T; 
}

void DynWBC::calcCostGrad()
{
    grad.setZero(contact_dim);
    grad -= base_contact_Jac.leftCols(6) * base_impedance_cmd;
}

void DynWBC::calcEqualityConstraint()
{

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
