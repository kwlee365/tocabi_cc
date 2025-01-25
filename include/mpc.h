#include "math_type_define.h"
#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include <iostream>

#include <casadi/casadi.hpp>
#include <vector>

#include <filesystem>

class MPC
{
    public:
        MPC(RobotData &rd, double initMpcFreq, double initN);
        RobotData &rd_;
        // ~MPC();      

        bool is_gradhess_init_ = true;
        bool is_mpc_init_ = true;

        int returnStateDim();
        int returnInputDim();

        void getRobotStateFromCC(const double &mass_cc, const Eigen::Matrix3d &inertia_cc, const Eigen::Vector3d &body_ori_cc, const Eigen::Vector3d &com_pos_cc, const Eigen::Vector3d &body_angvel_cc, const Eigen::Vector3d &com_linvel_cc);
        void casadiFunctionCall();
        void computeMPCGradientsHessian();
        void checkGradHessSize();
        void solveContactWrenchMPC();

        void setReferenceValue(const Eigen::MatrixXd &com_ref_cc, const Eigen::MatrixXd &com_dot_ref_cc, const Eigen::MatrixXd &body_euler_ref_cc, 
                               const Eigen::VectorXd &eta_l_ref_cc, const Eigen::VectorXd &eta_r_ref_cc, 
                               const Eigen::MatrixXd &lfoot_contact_point_ref_cc, const Eigen::MatrixXd &rfoot_contact_point_ref_cc,
                               const Eigen::MatrixXd &lfoot_contact_wrench_ref_cc, const Eigen::MatrixXd &rfoot_contact_wrench_ref_cc);
        void setWeightMatrix(const Eigen::VectorXd &W_Q_cc, const Eigen::VectorXd &W_R_cc);
     
        template <typename EigenType>
        void EigenToCasadiDM(casadi::DM &casadi_dm, const EigenType &eigen_data, int rows, int cols);
        template <typename ReturnType>
        ReturnType CasadiDMVectorToEigen(const std::vector<casadi::DM> &casadi_dm_vector);
        Eigen::VectorXd returnMPCControlInput() const;
        Eigen::MatrixXd returnPredictedState() const;
        Eigen::MatrixXd returnPredictedContactWrench() const;

        // QPOASES
        CQuadraticProgram QP_LMPC_SRBD_;
        Eigen::MatrixXd H_;  // HESSIAN
        Eigen::VectorXd g_;  // GRADIENT
        Eigen::MatrixXd A_;    
        Eigen::VectorXd lbA_;  
        Eigen::VectorXd ubA_;  
        int total_num_mpc_state = 0;
        int total_num_constraint = 0;
        Eigen::VectorXd v_mpc_;
        Eigen::VectorXd X_mpc_;
        Eigen::VectorXd U_mpc_;
        double dT_mpc = 0.0;

        // MPC STATES
        double mass_;
        double grav_;
        casadi::DM inertia_;
        casadi::DM x0;  // CURRENT ROBOT STATE
        casadi::DM X;   // OPT VARIABLES
        casadi::DM U;   // CTRL VARIABLES

        // MPC WEIGHTS
        casadi::DM W_Q;
        casadi::DM W_R;

        // MPC REFERENCE
        casadi::DM X_ref;
        casadi::DM U_ref;   
        casadi::DM com_ref_horizon;
        casadi::DM com_dot_ref_horizon;
        casadi::DM body_euler_ref_horizon;
        casadi::DM etaL_ref_horizon;
        casadi::DM etaR_ref_horizon;
        casadi::DM rL_ref_horizon;
        casadi::DM rR_ref_horizon;

        double com_height = 0.0;
        
        double f_z_max = 0.0;
        double f_z_min = 0.0;
        double mu = 0.0;

        double footX = 0.0;
        double footY = 0.0;

        std::string current_path = std::filesystem::current_path().parent_path().string();
        std::string library_path = current_path + "/catkin_ws/src/tocabi_cc/mpc_lib/";
        std::string library_name = "lib_mpc_func.so";

        casadi::Function J_v_func;         
        casadi::Function J_vv_func;       

        casadi::Function ceq1_func;
        casadi::Function ceq1_v_func;

        casadi::Function cineq1_max_func;
        casadi::Function cineq1_min_func;
        casadi::Function cineq2_max_func;
        casadi::Function cineq2_min_func;
        casadi::Function cineq3_max_func;
        casadi::Function cineq3_min_func;
        casadi::Function cineq4_max_func;
        casadi::Function cineq4_min_func;
        casadi::Function cineq5_max_func;
        casadi::Function cineq5_min_func;
        casadi::Function cineq1_max_v_func;
        casadi::Function cineq1_min_v_func;
        casadi::Function cineq2_max_v_func;
        casadi::Function cineq2_min_v_func;
        casadi::Function cineq3_max_v_func;
        casadi::Function cineq3_min_v_func;
        casadi::Function cineq4_max_v_func;
        casadi::Function cineq4_min_v_func;
        casadi::Function cineq5_max_v_func;
        casadi::Function cineq5_min_v_func;

    private:
        const double mpc_freq;
        const double mpc_N;
        const int state_length = 12;    // [theta_i, com_i, w_i, com_dot_i, grav] in R{13} 
        const int input_length = 12;    // [tau_l_i, tau_r_i, f_l_i, f_r_i] in R{12}
};