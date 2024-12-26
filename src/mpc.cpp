#include "mpc.h"

ofstream dataMPC_zmp_x("/home/kwan/catkin_ws/src/tocabi_cc/data/dataMPC_zmp_x.txt");
ofstream dataMPC_zmp_y("/home/kwan/catkin_ws/src/tocabi_cc/data/dataMPC_zmp_y.txt");
ofstream dataMPC_zmp_pred_x("/home/kwan/catkin_ws/src/tocabi_cc/data/dataMPC_zmp_pred_x.txt");
ofstream dataMPC_zmp_pred_y("/home/kwan/catkin_ws/src/tocabi_cc/data/dataMPC_zmp_pred_y.txt");
ofstream dataMPC_com_x("/home/kwan/catkin_ws/src/tocabi_cc/data/dataMPC_com_x.txt");
ofstream dataMPC_com_y("/home/kwan/catkin_ws/src/tocabi_cc/data/dataMPC_com_y.txt");
ofstream dataMPC_time("/home/kwan/catkin_ws/src/tocabi_cc/data/dataMPC_time.txt");

MPC::MPC(double initMpcFreq, double initN) : mpc_freq(initMpcFreq), N(initN)
{
    std::cout << "MPC FREQ: " << mpc_freq << "Hz" << std::endl;
    std::cout << "MPC HORIZON: " << N / mpc_freq << "s" << std::endl;
    std::cout << "MPC CLASS IS SUCCESSFULLY CONSTRUCTED" << std::endl;
}

void MPC::cartTableModel(double T, double h)
{
    A_lipm.setZero(3, 3);
    C_lipm.setZero(1, 3);
    B_lipm.setZero(3);

    A_lipm(0, 0) = 1.0;
    A_lipm(0, 1) = T;
    A_lipm(0, 2) = T * T / 2.0;
    A_lipm(1, 0) = 0.0;
    A_lipm(1, 1) = 1.0;
    A_lipm(1, 2) = T;
    A_lipm(2, 0) = 0.0;
    A_lipm(2, 1) = 0.0;
    A_lipm(2, 2) = 1.0;

    B_lipm(0) = T * T * T / 6.0;
    B_lipm(1) = T * T / 2.0;
    B_lipm(2) = T;

    C_lipm(0, 0) = 1.0;
    C_lipm(0, 1) = 0.0;
    C_lipm(0, 2) = -h / GRAVITY;
}

void MPC::cartTableModelMPC(double T, double h)
{
    cartTableModel(T, h);

    P_ps_lipm.setZero(N, 3);
    P_pu_lipm.setZero(N, N);
    P_vs_lipm.setZero(N, 3);
    P_vu_lipm.setZero(N, N);
    P_zs_lipm.setZero(N, 3);
    P_zu_lipm.setZero(N, N);

    for (int i = 0; i < N; i++)
    {
        P_ps_lipm(i, 0) = 1.0;
        P_ps_lipm(i, 1) = (i + 1) * T;
        P_ps_lipm(i, 2) = ((i + 1) * (i + 1) * T * T) / 2.0;

        P_vs_lipm(i, 0) = 0.0;
        P_vs_lipm(i, 1) = 1.0;
        P_vs_lipm(i, 2) = (i + 1) * T;

        P_zs_lipm(i, 0) = 1.0;
        P_zs_lipm(i, 1) = (i + 1) * T;
        P_zs_lipm(i, 2) = ((i + 1) * (i + 1) * T * T) / 2.0 - h / GRAVITY;

        for (int j = 0; j < N; j++)
        {
            if (j > i)
            {
                P_pu_lipm(i, j) = 0.0;
                P_vu_lipm(i, j) = 0.0;
                P_zu_lipm(i, j) = 0.0;
            }
            else
            {
                P_pu_lipm(i, j) = (1 + 3 * (i - j) + 3 * (i - j) * (i - j)) * (T * T * T) / 6.0;
                P_vu_lipm(i, j) = (1 + 2 * (i - j)) * (T * T) / 2.0;
                P_zu_lipm(i, j) = (1 + 3 * (i - j) + 3 * (i - j) * (i - j)) * (T * T * T) / 6.0 - (T * h / GRAVITY);
            }
        }
    }
}

void MPC::ComTrajectoryGenerator(Eigen::VectorXd &zx_ref, Eigen::VectorXd &zy_ref, Eigen::Vector3d &x_hat, Eigen::Vector3d &y_hat, double comHeight)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    double dT = 1.0 / mpc_freq;
    double w1 = 1e-6;
    double w2 = 1.0;

    if(is_mpc_init_ == true)
    {
        cartTableModelMPC(dT, comHeight);

        Hess_.setZero(N, N);

        grad_x.setZero(N);
        grad_y.setZero(N);

        lb_x.setZero(N);
        ub_x.setZero(N);
        lb_y.setZero(N);
        ub_y.setZero(N);

        U_x.setZero(N);
        U_y.setZero(N);

        Hess_ = w1 * Eigen::MatrixXd::Identity(N, N) + w2 * P_zu_lipm.transpose() * P_zu_lipm;

        QP_mpc_x_.InitializeProblemSize(N, N);
        QP_mpc_y_.InitializeProblemSize(N, N);

        is_mpc_init_ = false;

        std::cout << "SUCCESS TO INITIALIZE MPC PARAMETER" << std::endl;
    }

    grad_x = w2 * P_zu_lipm.transpose() * (P_zs_lipm * x_hat - zx_ref);
    grad_y = w2 * P_zu_lipm.transpose() * (P_zs_lipm * y_hat - zy_ref);

    lb_x = zx_ref - P_zs_lipm * x_hat - 0.13 * Eigen::VectorXd::Ones(N);
    ub_x = zx_ref - P_zs_lipm * x_hat + 0.17 * Eigen::VectorXd::Ones(N);
    lb_y = zy_ref - P_zs_lipm * y_hat - 0.10 * Eigen::VectorXd::Ones(N);
    ub_y = zy_ref - P_zs_lipm * y_hat + 0.10 * Eigen::VectorXd::Ones(N);

    Eigen::VectorXd U_x_temp;

    QP_mpc_x_.EnableEqualityCondition(1e-8);
    QP_mpc_x_.UpdateMinProblem(Hess_, grad_x);
    QP_mpc_x_.DeleteSubjectToAx();
    QP_mpc_x_.UpdateSubjectToAx(P_zu_lipm, lb_x, ub_x);

    if(QP_mpc_x_.SolveQPoases(500, U_x_temp))
    {
        U_x = U_x_temp.segment(0, N);

        x_hat = A_lipm * x_hat + B_lipm * U_x(0);
    }
    else
    {
        std::cout << "COM MPC X-AXIS SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
    }

    Eigen::VectorXd U_y_temp;

    QP_mpc_y_.EnableEqualityCondition(1e-8);
    QP_mpc_y_.UpdateMinProblem(Hess_, grad_y);
    QP_mpc_y_.DeleteSubjectToAx();
    QP_mpc_y_.UpdateSubjectToAx(P_zu_lipm, lb_y, ub_y);

    if(QP_mpc_y_.SolveQPoases(500, U_y_temp))
    {
        U_y = U_y_temp.segment(0, N);

        y_hat = A_lipm * y_hat + B_lipm * U_y(0);
    }
    else
    {
        std::cout << "COM MPC Y-AXIS SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

    dataMPC_zmp_x << zx_ref.transpose() << std::endl;
    dataMPC_zmp_y << zy_ref.transpose() << std::endl;
    dataMPC_zmp_pred_x << (P_zs_lipm * x_hat + P_zu_lipm * U_x).transpose() << std::endl;
    dataMPC_zmp_pred_y << (P_zs_lipm * y_hat + P_zu_lipm * U_y).transpose() << std::endl;
    dataMPC_com_x << (P_ps_lipm * x_hat + P_pu_lipm * U_x).transpose() << std::endl;
    dataMPC_com_y << (P_ps_lipm * y_hat + P_pu_lipm * U_y).transpose() << std::endl;
    dataMPC_time << duration.count() << std::endl;
}
