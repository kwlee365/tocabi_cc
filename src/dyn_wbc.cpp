// dyn_wbc.cpp
#include "dyn_wbc.h"
#include <iostream>

ofstream dataWBC("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWBC.txt");


using namespace Eigen;
using namespace qpOASES;

DynWBC::DynWBC(int dof) : dof_(dof) 
{
    std::cout << "CURRENT PATH: " << current_path << std::endl;
    std::cout << "LIBRARY PATH: " << library_path << std::endl;
    std::cout << "LIBRARY NAME: " << library_name << std::endl;
    std::cout << "LIBRARY: "      << library_path + library_name << std::endl;

    casadiFunctionCall();
}

Eigen::VectorQd DynWBC::computeDynamicWBC()
{
    //--- Initialization
    constraints_.clear();
    calcCostHess();
    calcCostGrad();
    calcEqualityConstraint();
    calcInequalityConstraint();

    if(is_wbc_init_ == true)
    {
        total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();
        for (const auto& c : constraints_) {total_num_constraints += c.A.rows();}

        //--- Initialization
        QP_Dyn_Wbc.InitializeProblemSize(total_num_state, total_num_constraints);

        A_   = Eigen::MatrixXd::Zero(total_num_constraints, total_num_state);
        lbA_ = Eigen::VectorXd::Zero(total_num_constraints);
        ubA_ = Eigen::VectorXd::Zero(total_num_constraints);

        torque_sol.setZero();
        contact_wrench_sol.setZero();

        is_wbc_init_ = false;
    }

    //--- Stack Constraints
    int row_idx  = 0;
    for (const auto& c : constraints_) {
        int rows = c.A.rows();
        A_.block(row_idx, 0, rows, total_num_state) = c.A;
        lbA_.segment(row_idx , rows)    = c.lbA;
        ubA_.segment(row_idx , rows)    = c.ubA;
        row_idx  += rows;
    }

    checkGradHessSize();

    QP_Dyn_Wbc.EnableEqualityCondition(1e-8);
    QP_Dyn_Wbc.UpdateMinProblem(Hess_, grad_);
    QP_Dyn_Wbc.DeleteSubjectToAx();
    QP_Dyn_Wbc.UpdateSubjectToAx(A_, lbA_, ubA_);

    Eigen::VectorXd F_; F_.setZero(total_num_state);
    if(QP_Dyn_Wbc.SolveQPoases(200, F_, true))
    {
        torque_sol = F_.head(MODEL_DOF);
        contact_wrench_sol = F_.tail(12);
    }
    else
    {
        // std::cout << "Dyn WBC SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
        torque_sol.setZero();
        contact_wrench_sol.setZero();
    }

    dataWBC << torque_sol.transpose() << " " << contact_wrench_sol.transpose() << std::endl;

    return(torque_sol);
}

//--- Functions
void DynWBC::casadiFunctionCall()
{
    std::string lib_full_name = library_path + library_name;

    J_v_func_          = casadi::external("J_v_func", lib_full_name);
    J_vv_func_         = casadi::external("J_vv_func", lib_full_name);

    ceq0_func_         = casadi::external("ceq0_func", lib_full_name);
    ceq0_v_func_       = casadi::external("ceq0_v_func", lib_full_name);

    ceq1_func_         = casadi::external("ceq1_func", lib_full_name);
    ceq1_v_func_       = casadi::external("ceq1_v_func", lib_full_name);

    cineq1_max_func_   = casadi::external("cineq1_max_func", lib_full_name);
    cineq1_max_v_func_ = casadi::external("cineq1_max_v_func", lib_full_name);
    cineq2_max_func_   = casadi::external("cineq2_max_func", lib_full_name);
    cineq2_max_v_func_ = casadi::external("cineq2_max_v_func", lib_full_name);
    cineq3_max_func_   = casadi::external("cineq3_max_func", lib_full_name);
    cineq3_max_v_func_ = casadi::external("cineq3_max_v_func", lib_full_name);
    cineq4_max_func_   = casadi::external("cineq4_max_func", lib_full_name);
    cineq4_max_v_func_ = casadi::external("cineq4_max_v_func", lib_full_name);
    cineq5_max_func_   = casadi::external("cineq5_max_func", lib_full_name);
    cineq5_max_v_func_ = casadi::external("cineq5_max_v_func", lib_full_name);
    cineq6_max_func_   = casadi::external("cineq6_max_func", lib_full_name);
    cineq6_max_v_func_ = casadi::external("cineq6_max_v_func", lib_full_name);
    cineq7_max_func_   = casadi::external("cineq7_max_func", lib_full_name);
    cineq7_max_v_func_ = casadi::external("cineq7_max_v_func", lib_full_name);

    cineq1_min_func_   = casadi::external("cineq1_min_func", lib_full_name);
    cineq1_min_v_func_ = casadi::external("cineq1_min_v_func", lib_full_name);
    cineq2_min_func_   = casadi::external("cineq2_min_func", lib_full_name);
    cineq2_min_v_func_ = casadi::external("cineq2_min_v_func", lib_full_name);
    cineq3_min_func_   = casadi::external("cineq3_min_func", lib_full_name);
    cineq3_min_v_func_ = casadi::external("cineq3_min_v_func", lib_full_name);
    cineq4_min_func_   = casadi::external("cineq4_min_func", lib_full_name);
    cineq4_min_v_func_ = casadi::external("cineq4_min_v_func", lib_full_name);
    cineq5_min_func_   = casadi::external("cineq5_min_func", lib_full_name);
    cineq5_min_v_func_ = casadi::external("cineq5_min_v_func", lib_full_name);
    cineq6_min_func_   = casadi::external("cineq6_min_func", lib_full_name);
    cineq6_min_v_func_ = casadi::external("cineq6_min_v_func", lib_full_name);
    cineq7_min_func_   = casadi::external("cineq7_min_func", lib_full_name);
    cineq7_min_v_func_ = casadi::external("cineq7_min_v_func", lib_full_name);

    std::cout << "CASADI FUNCTION CALL SUCCESS!" << std::endl;
}

void DynWBC::setRobotSystemParameters(const double& mu, const double& foot_size, const double& foot_width, const int& contact_dim, 
                                      const Eigen::VectorQd& torque_lim, const Eigen::VectorQd& q_pos_l_lim, const Eigen::VectorQd& q_pos_h_lim, 
                                      const double& force_z_max, const double& force_z_min)
{
    //--- Friction, Contact, Torque limit constraints
    mu_ = mu;
    foot_size_ = foot_size; 
    foot_width_ = foot_width; 
    contact_dim_ = contact_dim;
    force_z_max_ = force_z_max;
    force_z_min_ = force_z_min;

    EigenToCasadiDM<Eigen::VectorQd>(torque_lim_, torque_lim, torque_lim.size(), 1);
    EigenToCasadiDM<Eigen::VectorQd>(q_pos_l_lim_, q_pos_l_lim, q_pos_l_lim.size(), 1);
    EigenToCasadiDM<Eigen::VectorQd>(q_pos_h_lim_, q_pos_h_lim, q_pos_h_lim.size(), 1);
}

void DynWBC::setWbcWeights(const Eigen::VectorVQd& W_Q, const Eigen::VectorQd& W_torque, const Eigen::VectorXd& W_lambda, const Eigen::VectorQd& W_torque_prev)
{
    //--- Cost function
    EigenToCasadiDM<Eigen::VectorVQd>(W_Q_,      W_Q,      W_Q.size(), 1);
    EigenToCasadiDM<Eigen::VectorQd>( W_torque_, W_torque, W_torque.size(), 1);
    EigenToCasadiDM<Eigen::VectorXd>( W_lambda_, W_lambda, W_lambda.size(), 1);
    EigenToCasadiDM<Eigen::VectorQd>( W_torque_prev_, W_torque_prev, W_torque_prev.size(), 1);
}

void DynWBC::getRobotStates(const Eigen::MatrixVQVQd& H, 
                            const Eigen::VectorVQd& G, 
                            const Eigen::MatrixXd& J_c, 
                            const Eigen::VectorVQd& qddot_des_from_ik,
                            const Eigen::VectorQd& q,
                            const Eigen::VectorQd& qdot)
{
    EigenToCasadiDM<Eigen::MatrixVQVQd>(H_, H, H.rows(), H.cols());
    EigenToCasadiDM<Eigen::VectorVQd>(G_, G, G.size(), 1);
    EigenToCasadiDM<Eigen::MatrixXd>(J_c_, J_c, J_c.rows(), J_c.cols());
    EigenToCasadiDM<Eigen::VectorVQd>(qddot_des_from_ik_,  qddot_des_from_ik,  qddot_des_from_ik.size(), 1);

    Eigen::VectorQd torque; torque.setZero();
    Eigen::VectorXd lambda; lambda.setZero(J_c.rows());
    EigenToCasadiDM<Eigen::VectorQd>(torque_, torque, torque.size(), 1);
    EigenToCasadiDM<Eigen::VectorXd>(lambda_, lambda, lambda.size(), 1);

    EigenToCasadiDM<Eigen::VectorQd>(q_,       q,    q.size(), 1);
    EigenToCasadiDM<Eigen::VectorQd>(qdot_, qdot, qdot.size(), 1);

    EigenToCasadiDM<Eigen::VectorQd>(torque_sol_, torque_sol, torque_sol.size(), 1);
}

void DynWBC::calcCostHess()
{
    std::vector<casadi::DM> Hess_dm = J_vv_func_(std::vector<casadi::DM>{H_, G_, J_c_, qdot_, qddot_des_from_ik_, torque_, torque_sol_, lambda_, W_Q_, W_torque_, W_lambda_, W_torque_prev_});
    Hess_ = CasadiDMVectorToEigen<Eigen::MatrixXd>(Hess_dm);
}

void DynWBC::calcCostGrad()
{
    std::vector<casadi::DM> grad_dm = J_v_func_(std::vector<casadi::DM>{H_, G_, J_c_, qdot_, qddot_des_from_ik_, torque_, torque_sol_, lambda_, W_Q_, W_torque_, W_lambda_, W_torque_prev_});
    grad_ = CasadiDMVectorToEigen<Eigen::VectorXd>(grad_dm);
}

void DynWBC::calcEqualityConstraint()
{
    //--- Contact constraint
    // std::vector<casadi::DM> ceq0_dm   =   ceq0_func_(std::vector<casadi::DM>{H_, G_, J_c_, qddot_des_from_ik_, torque_, lambda_});
    // std::vector<casadi::DM> ceq0_v_dm = ceq0_v_func_(std::vector<casadi::DM>{H_, G_, J_c_, qddot_des_from_ik_, torque_, lambda_});

    // constraints_.push_back({CasadiDMVectorToEigen<Eigen::MatrixXd>(ceq0_v_dm), 
    //                (-1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(ceq0_dm), 
    //                (-1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(ceq0_dm)});

    //--- Virtual Joint Constraint
    std::vector<casadi::DM> ceq1_dm   =   ceq1_func_(std::vector<casadi::DM>{H_, G_, J_c_, qddot_des_from_ik_, lambda_});
    std::vector<casadi::DM> ceq1_v_dm = ceq1_v_func_(std::vector<casadi::DM>{H_, G_, J_c_, qddot_des_from_ik_, lambda_});

    constraints_.push_back({CasadiDMVectorToEigen<Eigen::MatrixXd>(ceq1_v_dm), 
                   (-1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(ceq1_dm), 
                   (-1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(ceq1_dm)});
}

void DynWBC::calcInequalityConstraint()
{
    //--- (1) CasADi Function Generation
    std::vector<casadi::DM> cineq1_max_dm = cineq1_max_func_(std::vector<casadi::DM>{lambda_, force_z_max_});
    std::vector<casadi::DM> cineq1_min_dm = cineq1_min_func_(std::vector<casadi::DM>{lambda_, force_z_min_});
    std::vector<casadi::DM> cineq2_max_dm = cineq2_max_func_(std::vector<casadi::DM>{lambda_, mu_});
    std::vector<casadi::DM> cineq2_min_dm = cineq2_min_func_(std::vector<casadi::DM>{lambda_, mu_});
    std::vector<casadi::DM> cineq3_max_dm = cineq3_max_func_(std::vector<casadi::DM>{lambda_, mu_});
    std::vector<casadi::DM> cineq3_min_dm = cineq3_min_func_(std::vector<casadi::DM>{lambda_, mu_});
    std::vector<casadi::DM> cineq4_max_dm = cineq4_max_func_(std::vector<casadi::DM>{lambda_, foot_width_ / 2.0});
    std::vector<casadi::DM> cineq4_min_dm = cineq4_min_func_(std::vector<casadi::DM>{lambda_, foot_width_ / 2.0});
    std::vector<casadi::DM> cineq5_max_dm = cineq5_max_func_(std::vector<casadi::DM>{lambda_, foot_size_  / 2.0});
    std::vector<casadi::DM> cineq5_min_dm = cineq5_min_func_(std::vector<casadi::DM>{lambda_, foot_size_  / 2.0});
    
    std::vector<casadi::DM> cineq1_max_v_dm = cineq1_max_v_func_(std::vector<casadi::DM>{lambda_, force_z_max_});
    std::vector<casadi::DM> cineq1_min_v_dm = cineq1_min_v_func_(std::vector<casadi::DM>{lambda_, force_z_min_});
    std::vector<casadi::DM> cineq2_max_v_dm = cineq2_max_v_func_(std::vector<casadi::DM>{lambda_, mu_});
    std::vector<casadi::DM> cineq2_min_v_dm = cineq2_min_v_func_(std::vector<casadi::DM>{lambda_, mu_});
    std::vector<casadi::DM> cineq3_max_v_dm = cineq3_max_v_func_(std::vector<casadi::DM>{lambda_, mu_});
    std::vector<casadi::DM> cineq3_min_v_dm = cineq3_min_v_func_(std::vector<casadi::DM>{lambda_, mu_});
    std::vector<casadi::DM> cineq4_max_v_dm = cineq4_max_v_func_(std::vector<casadi::DM>{lambda_, foot_width_ / 2.0});
    std::vector<casadi::DM> cineq4_min_v_dm = cineq4_min_v_func_(std::vector<casadi::DM>{lambda_, foot_width_ / 2.0});
    std::vector<casadi::DM> cineq5_max_v_dm = cineq5_max_v_func_(std::vector<casadi::DM>{lambda_, foot_size_  / 2.0});
    std::vector<casadi::DM> cineq5_min_v_dm = cineq5_min_v_func_(std::vector<casadi::DM>{lambda_, foot_size_  / 2.0});
    
    std::vector<casadi::DM> cineq6_max_dm = cineq6_max_func_(std::vector<casadi::DM>{torque_, torque_lim_});
    std::vector<casadi::DM> cineq6_min_dm = cineq6_min_func_(std::vector<casadi::DM>{torque_, torque_lim_});
    
    std::vector<casadi::DM> cineq6_max_v_dm = cineq6_max_v_func_(std::vector<casadi::DM>{torque_, torque_lim_});
    std::vector<casadi::DM> cineq6_min_v_dm = cineq6_min_v_func_(std::vector<casadi::DM>{torque_, torque_lim_});
    
    std::vector<casadi::DM> cineq7_max_dm = cineq7_max_func_(std::vector<casadi::DM>{H_, G_, J_c_, torque_, lambda_, q_, qdot_, q_pos_h_lim_, alpha1, alpha2});
    std::vector<casadi::DM> cineq7_min_dm = cineq7_min_func_(std::vector<casadi::DM>{H_, G_, J_c_, torque_, lambda_, q_, qdot_, q_pos_l_lim_, alpha1, alpha2});
    
    std::vector<casadi::DM> cineq7_max_v_dm = cineq7_max_v_func_(std::vector<casadi::DM>{H_, G_, J_c_, torque_, lambda_, q_, qdot_, q_pos_h_lim_, alpha1, alpha2});
    std::vector<casadi::DM> cineq7_min_v_dm = cineq7_min_v_func_(std::vector<casadi::DM>{H_, G_, J_c_, torque_, lambda_, q_, qdot_, q_pos_l_lim_, alpha1, alpha2});


    //--- (2) CasADi To Eigen
    Eigen::MatrixXd A1   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq1_max_v_dm);
    Eigen::VectorXd lbA1 = (+1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(cineq1_min_dm);
    Eigen::VectorXd ubA1 = (-1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(cineq1_max_dm);

    Eigen::MatrixXd A2_max = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq2_max_v_dm);
    Eigen::MatrixXd A2_min = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq2_min_v_dm);

    Eigen::MatrixXd A3_max = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq3_max_v_dm);
    Eigen::MatrixXd A3_min = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq3_min_v_dm);

    Eigen::MatrixXd A4_max = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq4_max_v_dm);
    Eigen::MatrixXd A4_min = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq4_min_v_dm);

    Eigen::MatrixXd A5_max = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq5_max_v_dm);
    Eigen::MatrixXd A5_min = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq5_min_v_dm);

    Eigen::MatrixXd A6 = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq6_max_v_dm);
    Eigen::VectorXd lbA6 = (+1.0) * CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq6_max_dm);
    Eigen::VectorXd ubA6 = (-1.0) * CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq6_min_dm);

    Eigen::MatrixXd A7 = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq7_max_v_dm);
    Eigen::VectorXd lbA7 = (+1.0) * CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq7_max_dm);
    Eigen::VectorXd ubA7 = (-1.0) * CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq7_min_dm);

    //--- (3) Stack Constraints 
    constraints_.push_back({A1, lbA1, ubA1});

    constraints_.push_back({
        A2_max,
        Eigen::VectorXd::Constant(A2_max.rows(), -std::numeric_limits<double>::infinity()),
        Eigen::VectorXd::Zero(A2_max.rows())
    });

    constraints_.push_back({
        A2_min,
        Eigen::VectorXd::Constant(A2_min.rows(), -std::numeric_limits<double>::infinity()),
        Eigen::VectorXd::Zero(A2_min.rows())
    });

    constraints_.push_back({
        A3_max,
        Eigen::VectorXd::Constant(A3_max.rows(), -std::numeric_limits<double>::infinity()),
        Eigen::VectorXd::Zero(A3_max.rows())
    });
    constraints_.push_back({
        A3_min,
        Eigen::VectorXd::Constant(A3_min.rows(), -std::numeric_limits<double>::infinity()),
        Eigen::VectorXd::Zero(A3_min.rows())
    });

    constraints_.push_back({
        A4_max,
        Eigen::VectorXd::Constant(A4_max.rows(), -std::numeric_limits<double>::infinity()),
        Eigen::VectorXd::Zero(A4_max.rows())
    });

    constraints_.push_back({
        A4_min,
        Eigen::VectorXd::Constant(A4_min.rows(), -std::numeric_limits<double>::infinity()),
        Eigen::VectorXd::Zero(A4_min.rows())
    });

    constraints_.push_back({
        A5_max,
        Eigen::VectorXd::Constant(A5_max.rows(), -std::numeric_limits<double>::infinity()),
        Eigen::VectorXd::Zero(A5_max.rows())
    });

    constraints_.push_back({
        A5_min,
        Eigen::VectorXd::Constant(A5_min.rows(), -std::numeric_limits<double>::infinity()),
        Eigen::VectorXd::Zero(A5_min.rows())
    });

    constraints_.push_back({A6, lbA6, ubA6});

    constraints_.push_back({A7, lbA7, ubA7});
}

void DynWBC::checkGradHessSize()
{
    if(is_gradhess_init_ == true)
    {
        std::cout << "==============================================" << std::endl;
        std::cout << "===== DynWBC COST & CONSTRAINTS DIM INFO =====" << std::endl;
        std::cout << "==============================================" << std::endl;

        std::cout << "Hess_ size: " << Hess_.rows() << " x " << Hess_.cols() << std::endl;
        std::cout << "grad_ size: " << grad_.size() << std::endl;
        std::cout << std::endl;

        std::cout << "A: " << A_.rows() << " x " << A_.cols() << std::endl;
        std::cout << "lbA size: " << lbA_.size() << std::endl;
        std::cout << "ubA size: " << ubA_.size() << std::endl;
        std::cout << std::endl;

        JointLimitChecker();

        // std::cout << "Hess_: " << Hess_ << std::endl;
        // std::cout << "grad_: " << grad_.transpose() << std::endl;
        // std::cout << std::endl;

        // std::cout << "A: " << A_ << std::endl;
        // std::cout << "lbA: " << lbA_.transpose() << std::endl;
        // std::cout << "ubA: " << ubA_.transpose() << std::endl;
        std::cout << std::endl;

        // std::cout << "==== INPUT CHECK ====" << std::endl;
        // std::cout << "H_: \n" << H_ << std::endl;
        // std::cout << "G_: \n" << G_ << std::endl;
        // std::cout << "J_c_: \n" << J_c_ << std::endl;
        // std::cout << "qddot_des_from_ik_: \n" << qddot_des_from_ik_ << std::endl;
        // std::cout << "torque_: \n" << torque_ << std::endl;
        // std::cout << "lambda_: \n" << lambda_ << std::endl;

        is_gradhess_init_ = false;
    }

}


//--- Utils
template <typename EigenType>
void DynWBC::EigenToCasadiDM(casadi::DM &casadi_dm, const EigenType &eigen_data, int rows, int cols)
{
    casadi_dm = casadi::DM::zeros(rows, cols);
    memcpy(casadi_dm.ptr(), eigen_data.data(), sizeof(double) * rows * cols);
}

template <typename ReturnType>
ReturnType DynWBC::CasadiDMVectorToEigen(const std::vector<casadi::DM> &casadi_dm_vector)
{
    casadi::DM Matrx = casadi_dm_vector.at(0);
    casadi::Sparsity SpA = Matrx.get_sparsity();

    std::vector<casadi_int> output_row, output_col;
    SpA.get_triplet(output_row, output_col);
    std::vector<double> values = Matrx.get_nonzeros();

    using T = Eigen::Triplet<double>;
    std::vector<T> TripletList;
    TripletList.resize(values.size());
    for(int k = 0; k < values.size(); ++k)
        TripletList[k] = T(output_row[k], output_col[k], values[k]);

    Eigen::SparseMatrix<double> SpMatrx(Matrx.size1(), Matrx.size2());
    SpMatrx.setFromTriplets(TripletList.begin(), TripletList.end());

    if constexpr (std::is_same<ReturnType, Eigen::MatrixXd>::value) 
    {
        return Eigen::MatrixXd(SpMatrx);
    } 
    else if constexpr (std::is_same<ReturnType, Eigen::VectorXd>::value) 
    {
        Eigen::MatrixXd temp_mat = Eigen::MatrixXd(SpMatrx);
        return Eigen::VectorXd(Eigen::Map<Eigen::VectorXd>(temp_mat.data(), temp_mat.cols() * temp_mat.rows()));
    } 
    else 
    {
        static_assert("Unsupported ReturnType. Use Eigen::MatrixXd or Eigen::VectorXd.");
    }
}

void DynWBC::JointLimitChecker()
{
    //--- Joint Limit Violation Checker
    std::vector<int> violated_indices;

    for (int i = 0; i < q_.size1(); ++i)
    {
        double qi    = static_cast<double>(q_(i));
        double q_min = static_cast<double>(q_pos_l_lim_(i));
        double q_max = static_cast<double>(q_pos_h_lim_(i));

        if (qi < q_min || qi > q_max)
        {
            violated_indices.push_back(i);
            std::cerr << "[JOINT LIMIT VIOLATION] Joint " << i
                    << " = " << qi
                    << " (Limit: " << q_min << " ~ " << q_max << ")"
                    << std::endl;
        }
    }

    if (violated_indices.empty())
    {
        std::cout << "[JOINT LIMIT CHECK] All joints are within limits." << std::endl;
    }
    else
    {
        std::cout << "[JOINT LIMIT CHECK] Violated joints: ";
        for (int idx : violated_indices)
        {
            std::cout << idx << " ";
        }
        std::cout << std::endl;
    }
}