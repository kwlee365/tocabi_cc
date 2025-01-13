#include "math_type_define.h"
#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"

#include <casadi/casadi.hpp>
#include <vector>

class MPC
{
    public:
        MPC(RobotData &rd, double initMpcFreq, double initN);
        RobotData &rd_;
        // ~MPC();      

        bool is_param_init_ = true;

        void parameterSRBD();
        void modelSRBD();

        double mass_;
        Eigen::Matrix3d inertia_;

        // QPOASES
        CQuadraticProgram QP_mpc_x_;
        CQuadraticProgram QP_mpc_y_;
        Eigen::VectorXd U_x;
        Eigen::VectorXd U_y;
    private:
        const double mpc_freq;
        const double N;
        const int state_length = 12;    // [theta_i, com_i, w_i, com_dot_i] \in R^{12} 
        const int input_length = 12;    // [tau_l_i, tau_r_i, f_l_i, f_r_i] \in R^{12}
};