#include "math_type_define.h"
#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"

class MPC
{
    public:
        MPC(double initMpcFreq, double initN);
        // ~MPC();

        void cartTableModel(double T, double h);
        void cartTableModelMPC(double T, double com_height);
        void ComTrajectoryGenerator(Eigen::VectorXd &zx_ref, Eigen::VectorXd &zy_ref, Eigen::Vector3d &x_hat, Eigen::Vector3d &y_hat, double comHeight);

        bool is_mpc_init_ = true;

        Eigen::MatrixXd A_lipm, C_lipm;
        Eigen::VectorXd B_lipm;
        Eigen::MatrixXd P_ps_lipm, P_pu_lipm, P_vs_lipm, P_vu_lipm, P_zs_lipm, P_zu_lipm;
        
        Eigen::MatrixXd Hess_;
        Eigen::VectorXd grad_x;
        Eigen::VectorXd grad_y;
        Eigen::VectorXd lb_x;
        Eigen::VectorXd ub_x;
        Eigen::VectorXd lb_y;
        Eigen::VectorXd ub_y;

        // QPOASES
        CQuadraticProgram QP_mpc_x_;
        CQuadraticProgram QP_mpc_y_;
        Eigen::VectorXd U_x;
        Eigen::VectorXd U_y;
    private:
        const double mpc_freq;
        const double N;
};