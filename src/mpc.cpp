#include "mpc.h"

ofstream dataMPC("/home/kwan/catkin_ws/src/tocabi_cc/data/dataMPC.txt");

using namespace casadi;

MPC::MPC(RobotData &rd, double initMpcFreq, double initN) : rd_(rd), mpc_freq(initMpcFreq), mpc_N(initN)
{
    std::cout << "CURRENT PATH: " << current_path << std::endl;
    std::cout << "LIBRARY PATH: " << library_path << std::endl;
    std::cout << "LIBRARY NAME: " << library_name << std::endl;
    std::cout << "LIBRARY: "      << library_path + library_name << std::endl;

    casadiFunctionCall();
}
int MPC::returnStateDim() {return state_length;}

int MPC::returnInputDim() {return input_length;}

void MPC::casadiFunctionCall()
{
    std::string lib_full_name = library_path + library_name;

    J_v_func          = casadi::external("J_v_func", lib_full_name);
    J_vv_func         = casadi::external("J_vv_func", lib_full_name);

    ceq1_func   = casadi::external("ceq1_func", lib_full_name);
    ceq1_v_func = casadi::external("ceq1_v_func", lib_full_name);

    cineq1_max_func   = casadi::external("cineq1_max_func", lib_full_name);
    cineq1_min_func   = casadi::external("cineq1_min_func", lib_full_name);
    cineq2_max_func   = casadi::external("cineq2_max_func", lib_full_name);
    cineq2_min_func   = casadi::external("cineq2_min_func", lib_full_name);
    cineq3_max_func   = casadi::external("cineq3_max_func", lib_full_name);
    cineq3_min_func   = casadi::external("cineq3_min_func", lib_full_name);
    cineq4_max_func   = casadi::external("cineq4_max_func", lib_full_name);
    cineq4_min_func   = casadi::external("cineq4_min_func", lib_full_name);
    cineq5_max_func   = casadi::external("cineq5_max_func", lib_full_name);
    cineq5_min_func   = casadi::external("cineq5_min_func", lib_full_name);
    cineq1_max_v_func = casadi::external("cineq1_max_v_func", lib_full_name);
    cineq1_min_v_func = casadi::external("cineq1_min_v_func", lib_full_name);
    cineq2_max_v_func = casadi::external("cineq2_max_v_func", lib_full_name);
    cineq2_min_v_func = casadi::external("cineq2_min_v_func", lib_full_name);
    cineq3_max_v_func = casadi::external("cineq3_max_v_func", lib_full_name);
    cineq3_min_v_func = casadi::external("cineq3_min_v_func", lib_full_name);
    cineq4_max_v_func = casadi::external("cineq4_max_v_func", lib_full_name);
    cineq4_min_v_func = casadi::external("cineq4_min_v_func", lib_full_name);
    cineq5_max_v_func = casadi::external("cineq5_max_v_func", lib_full_name);
    cineq5_min_v_func = casadi::external("cineq5_min_v_func", lib_full_name);

    std::cout << "CASADI FUNCTION CALL SUCCESS!" << std::endl;
}

void MPC::getRobotStateFromCC(const double &mass_cc, const Eigen::Matrix3d &inertia_cc, const Eigen::Vector3d &body_ori_cc, const Eigen::Vector3d &com_pos_cc, const Eigen::Vector3d &body_angvel_cc, const Eigen::Vector3d &com_linvel_cc)
{
    mass_ = mass_cc;
    grav_ = GRAVITY;
    EigenToCasadiDM<Eigen::MatrixXd>(inertia_, inertia_cc, inertia_cc.rows(), inertia_cc.cols());

    f_z_max = 1.2 * mass_ * grav_;
    f_z_min = 0.0;
    mu = 0.7;

    footX = 0.30;
    footY = 0.18;

    dT_mpc = 1.0 / mpc_freq;

    Eigen::VectorXd x0_cc; x0_cc.setZero(state_length);
    // x0_cc.segment(0, 3) = body_ori_cc;
    x0_cc.segment(3, 3) = com_pos_cc;
    // x0_cc.segment(6, 3) = body_angvel_cc;
    x0_cc.segment(9, 3) = com_linvel_cc;
    EigenToCasadiDM<Eigen::VectorXd>(x0, x0_cc, x0_cc.size(), 1);
    
    v_mpc_.setZero(total_num_mpc_state);
    X_mpc_.setZero(state_length * mpc_N); EigenToCasadiDM<Eigen::VectorXd>(X, X_mpc_, X_mpc_.size(), 1);
    U_mpc_.setZero(input_length * mpc_N); EigenToCasadiDM<Eigen::VectorXd>(U, U_mpc_, U_mpc_.size(), 1);
}

void MPC::setReferenceValue(const Eigen::MatrixXd &com_ref_cc, const Eigen::MatrixXd &com_dot_ref_cc, const Eigen::MatrixXd &body_euler_ref_cc, 
                            const Eigen::VectorXd &eta_l_ref_cc, const Eigen::VectorXd &eta_r_ref_cc, 
                            const Eigen::MatrixXd &lfoot_contact_point_ref_cc, const Eigen::MatrixXd &rfoot_contact_point_ref_cc,
                            const Eigen::MatrixXd &lfoot_contact_wrench_ref_cc, const Eigen::MatrixXd &rfoot_contact_wrench_ref_cc)
{
    Eigen::VectorXd X_ref_cc; X_ref_cc.setZero(state_length * mpc_N);
    for (int i = 0; i < mpc_N; i++)
    {
        X_ref_cc.segment(state_length * i + 0, 3) = Eigen::Vector3d::Zero();   
        X_ref_cc.segment(state_length * i + 3, 3) = com_ref_cc.col(i);
        X_ref_cc.segment(state_length * i + 6, 3) = Eigen::Vector3d::Zero();
        X_ref_cc.segment(state_length * i + 9, 3) = com_dot_ref_cc.col(i);
    }
    EigenToCasadiDM<Eigen::VectorXd>(X_ref, X_ref_cc, X_ref_cc.size(), 1);

    Eigen::VectorXd U_ref_cc; U_ref_cc.setZero(input_length * mpc_N);
    for (int i = 0; i < mpc_N; i++)
    {
        U_ref_cc.segment(input_length * i + 0, 6) = lfoot_contact_wrench_ref_cc.col(i);   
        U_ref_cc.segment(input_length * i + 6, 6) = rfoot_contact_wrench_ref_cc.col(i);
    }
    EigenToCasadiDM<Eigen::VectorXd>(U_ref, U_ref_cc, U_ref_cc.size(), 1);

    EigenToCasadiDM<Eigen::MatrixXd>(com_ref_horizon, com_ref_cc, com_ref_cc.rows(), com_ref_cc.cols());
    EigenToCasadiDM<Eigen::MatrixXd>(com_dot_ref_horizon, com_dot_ref_cc, com_dot_ref_cc.rows(), com_dot_ref_cc.cols());
    EigenToCasadiDM<Eigen::MatrixXd>(body_euler_ref_horizon, body_euler_ref_cc, body_euler_ref_cc.rows(), body_euler_ref_cc.cols());

    EigenToCasadiDM<Eigen::VectorXd>(etaL_ref_horizon, eta_l_ref_cc, eta_l_ref_cc.size(), 1); 
    EigenToCasadiDM<Eigen::VectorXd>(etaR_ref_horizon, eta_r_ref_cc, eta_r_ref_cc.size(), 1);
    EigenToCasadiDM<Eigen::MatrixXd>(rL_ref_horizon, lfoot_contact_point_ref_cc - com_ref_cc, lfoot_contact_point_ref_cc.rows(), lfoot_contact_point_ref_cc.cols());
    EigenToCasadiDM<Eigen::MatrixXd>(rR_ref_horizon, rfoot_contact_point_ref_cc - com_ref_cc, rfoot_contact_point_ref_cc.rows(), rfoot_contact_point_ref_cc.cols());
}

void MPC::setWeightMatrix(const Eigen::VectorXd &W_Q_cc, const Eigen::VectorXd &W_R_cc)
{
    EigenToCasadiDM<Eigen::VectorXd>(W_Q, W_Q_cc, W_Q_cc.size(), 1);
    EigenToCasadiDM<Eigen::VectorXd>(W_R, W_R_cc, W_R_cc.size(), 1);
}

void MPC::computeMPCGradientsHessian()
{
    std::vector<casadi::DM> H_dm = J_vv_func(std::vector<casadi::DM>{X, U, X_ref, U_ref, W_Q, W_R});
    std::vector<casadi::DM> g_dm = J_v_func( std::vector<casadi::DM>{X, U, X_ref, U_ref, W_Q, W_R});
    
    std::vector<casadi::DM> ceq1_dm   =   ceq1_func(std::vector<casadi::DM>{x0, X, U, mass_, grav_, inertia_, dT_mpc, rL_ref_horizon, rR_ref_horizon, body_euler_ref_horizon, etaL_ref_horizon, etaR_ref_horizon});
    std::vector<casadi::DM> ceq1_v_dm = ceq1_v_func(std::vector<casadi::DM>{x0, X, U, mass_, grav_, inertia_, dT_mpc, rL_ref_horizon, rR_ref_horizon, body_euler_ref_horizon, etaL_ref_horizon, etaR_ref_horizon});

    // std::cout << "x0: " << x0 << std::endl;
    // std::cout << "X: " << X << std::endl;
    // std::cout << "U: " << U << std::endl;
    // std::cout << "mass: " << mass_ << std::endl;
    // std::cout << "grav: " << grav_ << std::endl;
    // std::cout << "inertia_: " << inertia_ << std::endl;
    // std::cout << "dT_mpc: " << dT_mpc << std::endl;
    // std::cout << "rL_ref_horizon: " << rL_ref_horizon << std::endl;
    // std::cout << "rR_ref_horizon: " << rR_ref_horizon << std::endl;
    // std::cout << "body_euler_ref_horizon: " << body_euler_ref_horizon << std::endl;
    // std::cout << "etaL_ref_horizon: " << etaL_ref_horizon << std::endl;
    // std::cout << "etaR_ref_horizon: " << etaR_ref_horizon << std::endl;

    std::vector<casadi::DM> cineq1_max_dm = cineq1_max_func(std::vector<casadi::DM>{U, f_z_max});   
    std::vector<casadi::DM> cineq1_min_dm = cineq1_min_func(std::vector<casadi::DM>{U, f_z_min});  
    std::vector<casadi::DM> cineq2_max_dm = cineq2_max_func(std::vector<casadi::DM>{U, mu});
    std::vector<casadi::DM> cineq2_min_dm = cineq2_min_func(std::vector<casadi::DM>{U, mu});
    std::vector<casadi::DM> cineq3_max_dm = cineq3_max_func(std::vector<casadi::DM>{U, mu});
    std::vector<casadi::DM> cineq3_min_dm = cineq3_min_func(std::vector<casadi::DM>{U, mu});
    std::vector<casadi::DM> cineq4_max_dm = cineq4_max_func(std::vector<casadi::DM>{U, footY / 2.0});
    std::vector<casadi::DM> cineq4_min_dm = cineq4_min_func(std::vector<casadi::DM>{U, footY / 2.0});
    std::vector<casadi::DM> cineq5_max_dm = cineq5_max_func(std::vector<casadi::DM>{U, footX / 2.0});
    std::vector<casadi::DM> cineq5_min_dm = cineq5_min_func(std::vector<casadi::DM>{U, footX / 2.0});

    std::vector<casadi::DM> cineq1_max_v_dm = cineq1_max_v_func(std::vector<casadi::DM>{U, f_z_max});
    std::vector<casadi::DM> cineq1_min_v_dm = cineq1_min_v_func(std::vector<casadi::DM>{U, f_z_min});
    std::vector<casadi::DM> cineq2_max_v_dm = cineq2_max_v_func(std::vector<casadi::DM>{U, mu});
    std::vector<casadi::DM> cineq2_min_v_dm = cineq2_min_v_func(std::vector<casadi::DM>{U, mu});
    std::vector<casadi::DM> cineq3_max_v_dm = cineq3_max_v_func(std::vector<casadi::DM>{U, mu});
    std::vector<casadi::DM> cineq3_min_v_dm = cineq3_min_v_func(std::vector<casadi::DM>{U, mu});
    std::vector<casadi::DM> cineq4_max_v_dm = cineq4_max_v_func(std::vector<casadi::DM>{U, footY / 2.0});
    std::vector<casadi::DM> cineq4_min_v_dm = cineq4_min_v_func(std::vector<casadi::DM>{U, footY / 2.0});
    std::vector<casadi::DM> cineq5_max_v_dm = cineq5_max_v_func(std::vector<casadi::DM>{U, footX / 2.0});
    std::vector<casadi::DM> cineq5_min_v_dm = cineq5_min_v_func(std::vector<casadi::DM>{U, footX / 2.0});

    // COST FUNCTION
    H_ = CasadiDMVectorToEigen<Eigen::MatrixXd>(H_dm);
    g_ = CasadiDMVectorToEigen<Eigen::VectorXd>(g_dm);

    // EQUALITY CONSTRAINTS 
    Eigen::MatrixXd Aeq1   = CasadiDMVectorToEigen<Eigen::MatrixXd>(ceq1_v_dm);
    Eigen::VectorXd lbAeq1 = (-1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(ceq1_dm);
    Eigen::VectorXd ubAeq1 = (-1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(ceq1_dm);

    // INEQUALITY CONSTRAINTS 
    // UNILATERAL CONTACT CONDITION
    Eigen::MatrixXd A1   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq1_max_v_dm);
    Eigen::VectorXd lbA1 = (+1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(cineq1_min_dm);
    Eigen::VectorXd ubA1 = (-1.0) * CasadiDMVectorToEigen<Eigen::VectorXd>(cineq1_max_dm);
    
    // NO SLIP CONDITION (HORIZONTAL FORCE, X)
    Eigen::MatrixXd A2_max   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq2_max_v_dm);
    Eigen::MatrixXd A2_min   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq2_min_v_dm);

    // NO SLIP CONDITION (HORIZONTAL FORCE, Y)
    Eigen::MatrixXd A3_max   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq3_max_v_dm);
    Eigen::MatrixXd A3_min   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq3_min_v_dm);

    // NO TIPPING CONDITION (HORIZONTAL MOMENT, X)
    Eigen::MatrixXd A4_max   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq4_max_v_dm);
    Eigen::MatrixXd A4_min   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq4_min_v_dm);

    // NO TIPPING CONDITION (HORIZONTAL MOMENT, Y)
    Eigen::MatrixXd A5_max   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq5_max_v_dm);
    Eigen::MatrixXd A5_min   = CasadiDMVectorToEigen<Eigen::MatrixXd>(cineq5_min_v_dm);

    int ceq1_dim   = Aeq1.rows();
    int cineq1_dim = A1.rows();
    int cineq2_max_dim = A2_max.rows(); int cineq2_min_dim = A2_min.rows();
    int cineq3_max_dim = A3_max.rows(); int cineq3_min_dim = A3_min.rows();
    int cineq4_max_dim = A4_max.rows(); int cineq4_min_dim = A4_min.rows();
    int cineq5_max_dim = A5_max.rows(); int cineq5_min_dim = A5_min.rows();

    total_num_mpc_state = (state_length + input_length) * mpc_N;
    total_num_constraint = ceq1_dim + cineq1_dim + cineq2_max_dim + cineq2_min_dim 
                                                 + cineq3_max_dim + cineq3_min_dim
                                                 + cineq4_max_dim + cineq4_min_dim
                                                 + cineq5_max_dim + cineq5_min_dim;

    int stack_cnt = 0;
    A_.setZero(total_num_constraint, total_num_mpc_state);
    lbA_.setZero(total_num_constraint);
    ubA_.setZero(total_num_constraint);

    A_.block(stack_cnt, 0,  ceq1_dim, total_num_mpc_state) = Aeq1;  
    lbA_.segment(stack_cnt, ceq1_dim) = lbAeq1;
    ubA_.segment(stack_cnt, ceq1_dim) = ubAeq1;
    stack_cnt += ceq1_dim;

    A_.block(stack_cnt, 0,  cineq1_dim, total_num_mpc_state) = A1;  
    lbA_.segment(stack_cnt, cineq1_dim) = lbA1;
    ubA_.segment(stack_cnt, cineq1_dim) = ubA1;
    stack_cnt += cineq1_dim;

    A_.block(stack_cnt, 0,  cineq2_max_dim, total_num_mpc_state) = A2_max;  
    lbA_.segment(stack_cnt, cineq2_max_dim).setConstant(-std::numeric_limits<double>::infinity());;
    ubA_.segment(stack_cnt, cineq2_max_dim).setZero();
    stack_cnt += cineq2_max_dim;
   
    A_.block(stack_cnt, 0, cineq2_min_dim, total_num_mpc_state) = A2_min;
    lbA_.segment(stack_cnt, cineq2_min_dim).setConstant(-std::numeric_limits<double>::infinity());
    ubA_.segment(stack_cnt, cineq2_min_dim).setZero();
    stack_cnt += cineq2_min_dim;

    A_.block(stack_cnt, 0, cineq3_max_dim, total_num_mpc_state) = A3_max;
    lbA_.segment(stack_cnt, cineq3_max_dim).setConstant(-std::numeric_limits<double>::infinity());
    ubA_.segment(stack_cnt, cineq3_max_dim).setZero();
    stack_cnt += cineq3_max_dim;

    A_.block(stack_cnt, 0, cineq3_min_dim, total_num_mpc_state) = A3_min;
    lbA_.segment(stack_cnt, cineq3_min_dim).setConstant(-std::numeric_limits<double>::infinity());
    ubA_.segment(stack_cnt, cineq3_min_dim).setZero();
    stack_cnt += cineq3_min_dim;

    A_.block(stack_cnt, 0, cineq4_max_dim, total_num_mpc_state) = A4_max;
    lbA_.segment(stack_cnt, cineq4_max_dim).setConstant(-std::numeric_limits<double>::infinity());
    ubA_.segment(stack_cnt, cineq4_max_dim).setZero();
    stack_cnt += cineq4_max_dim;

    A_.block(stack_cnt, 0, cineq4_min_dim, total_num_mpc_state) = A4_min;
    lbA_.segment(stack_cnt, cineq4_min_dim).setConstant(-std::numeric_limits<double>::infinity());
    ubA_.segment(stack_cnt, cineq4_min_dim).setZero();
    stack_cnt += cineq4_min_dim;

    A_.block(stack_cnt, 0, cineq5_max_dim, total_num_mpc_state) = A5_max;
    lbA_.segment(stack_cnt, cineq5_max_dim).setConstant(-std::numeric_limits<double>::infinity());
    ubA_.segment(stack_cnt, cineq5_max_dim).setZero();
    stack_cnt += cineq5_max_dim;

    A_.block(stack_cnt, 0, cineq5_min_dim, total_num_mpc_state) = A5_min;
    lbA_.segment(stack_cnt, cineq5_min_dim).setConstant(-std::numeric_limits<double>::infinity());
    ubA_.segment(stack_cnt, cineq5_min_dim).setZero();
    stack_cnt += cineq5_min_dim;

    checkGradHessSize();
}

void MPC::checkGradHessSize()
{
    if(is_gradhess_init_ == true)
    {
        std::cout << "================================================" << std::endl;
        std::cout << "===== SRBD-MPC COST & CONSTRAINTS DIM INFO =====" << std::endl;
        std::cout << "================================================" << std::endl;

        std::cout << "H_: " << H_.rows() << " x " << H_.cols() << std::endl;
        std::cout << "g_ size: " << g_.size() << std::endl;
        std::cout << std::endl;

        std::cout << "A: " << A_.rows() << " x " << A_.cols() << std::endl;
        std::cout << "lbA size: " << lbA_.size() << std::endl;
        std::cout << "ubA size: " << ubA_.size() << std::endl;
        std::cout << std::endl;
  
        dataMPC << "H_: " << H_ << std::endl;
        dataMPC << "g_: " << g_.transpose() << std::endl;
        dataMPC << std::endl;

        dataMPC << "A: " << A_ << std::endl;
        dataMPC << "lbA: " << lbA_.transpose() << std::endl;
        dataMPC << "ubA: " << ubA_.transpose() << std::endl;
        dataMPC << std::endl;

        is_gradhess_init_ = false;
    }
}


void MPC::solveContactWrenchMPC()
{
    computeMPCGradientsHessian();

    if(is_mpc_init_ == true)
    {
        QP_LMPC_SRBD_.InitializeProblemSize(total_num_mpc_state, total_num_constraint);

        v_mpc_.setZero(total_num_mpc_state);

        std::cout << "SRBD MPC CLASS IS SUCCESSFULLY INITIALIZED" << std::endl;
        std::cout << "OPTIMIZATION VARIABLES NUMBER: " << total_num_mpc_state << std::endl;
        std::cout << "OPTIMIZATION CONSTRAINTS NUMBER: " << total_num_constraint << std::endl;
        std::cout << "MPC FREQ: " << mpc_freq << "Hz" << std::endl;
        std::cout << "MPC HORIZON: " << mpc_N / mpc_freq << "s" << std::endl;
        is_mpc_init_ = false;
    }

    QP_LMPC_SRBD_.EnableEqualityCondition(1e-8);
    QP_LMPC_SRBD_.EnableMaxCpuTime(dT_mpc);
    QP_LMPC_SRBD_.UpdateMinProblem(H_, g_);
    QP_LMPC_SRBD_.DeleteSubjectToAx();
    QP_LMPC_SRBD_.UpdateSubjectToAx(A_, lbA_, ubA_);

    Eigen::VectorXd v_temp_; v_temp_.setZero(total_num_mpc_state);
    if(QP_LMPC_SRBD_.SolveQPoases(200, v_temp_, true))
    {
        v_mpc_ = v_temp_.segment(0, total_num_mpc_state);
        X_mpc_ = v_mpc_.segment(0                   , state_length * mpc_N);
        U_mpc_ = v_mpc_.segment(state_length * mpc_N, input_length * mpc_N);
    }
    else
    {
        std::cout << "SRBD MPC SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
        v_mpc_.setZero(total_num_mpc_state);
        X_mpc_.setZero(state_length * mpc_N);
        U_mpc_.setZero(input_length * mpc_N);
    }
}

Eigen::VectorXd MPC::returnMPCControlInput() const
{
    return U_mpc_.segment(0, input_length);
}

Eigen::MatrixXd MPC::returnPredictedState() const
{
    Eigen::MatrixXd state_pred_from_mpc; state_pred_from_mpc.setZero(state_length, mpc_N);
    for (int i = 0; i < mpc_N; i++)
    {
        state_pred_from_mpc.block(0, i, state_length, 1) = X_mpc_.segment(state_length * i, state_length);
    }

    return state_pred_from_mpc;
}

Eigen::MatrixXd MPC::returnPredictedContactWrench() const
{
    Eigen::MatrixXd contact_wrench_pred_horizon; contact_wrench_pred_horizon.setZero(input_length, mpc_N);

    for (int i = 0; i < mpc_N; i++)
    {
        contact_wrench_pred_horizon.block(0, i, input_length, 1) = U_mpc_.segment(input_length * i, input_length);
    }

    return contact_wrench_pred_horizon;
}


template <typename EigenType>
void MPC::EigenToCasadiDM(casadi::DM &casadi_dm, const EigenType &eigen_data, int rows, int cols)
{
    casadi_dm = casadi::DM::zeros(rows, cols);
    memcpy(casadi_dm.ptr(), eigen_data.data(), sizeof(double) * rows * cols);
}

template <typename ReturnType>
ReturnType MPC::CasadiDMVectorToEigen(const std::vector<casadi::DM> &casadi_dm_vector)
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
