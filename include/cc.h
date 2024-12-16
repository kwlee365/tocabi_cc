#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include <std_msgs/String.h>
#include "math_type_define.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include "tocabi_msgs/matrix_3_4.h"
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    void computeSlow();
    void computeFast();
    void computeThread3();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;

    // ROBOT 
    RigidBodyDynamics::Model model_d_;  //updated by desired q
    RigidBodyDynamics::Model model_c_;  //updated by current q
    void getParameterYAML();

    // LOW-LEVEL CONTROL
    void setGains();
    void moveInitialPose();
    Eigen::VectorQd q_init_;
    Eigen::VectorQd q_ref_;
    Eigen::VectorQd q_prev_;
    Eigen::Vector12d q_leg_desired_;
    Eigen::VectorQd Kp;
    Eigen::MatrixQQd Kp_diag;
    Eigen::VectorQd Kd;
    Eigen::MatrixQQd Kd_diag;

    Eigen::VectorXd joint_limit_l_;
    Eigen::VectorXd joint_limit_h_;
    Eigen::VectorXd joint_vel_limit_l_;
    Eigen::VectorXd joint_vel_limit_h_;

    // BOOLEAN OPERATOR
    bool walking_enable_;
    bool is_mode_6_init = true;
    bool is_mode_7_init = true;
    bool is_joystick_mode = false;
    bool is_support_foot_change = false;    
    bool is_lfoot_support = false;
    bool is_rfoot_support = false;
    bool is_dsp1 = false;
    bool is_ssp  = false;
    bool is_dsp2 = false;
    bool is_preview_ctrl_init = true;
    std::atomic<bool> atb_grav_update_{false};

    // BIPED WALKING PARAMETER
    void walkingParameterSetting();
    double target_x_;
    double target_y_;
    double target_z_;
    double target_theta_ = 0.0;
    double step_length_x_;
    double step_length_y_;
    double com_height_;
    double foot_height_;
    int is_right_foot_swing_;

    double t_last_;
    double t_start_;
    double t_start_real_;
    double t_temp_;  
    double t_rest_init_;
    double t_rest_last_;
    double t_double1_;
    double t_double2_;

    double t_total_;
    double t_dsp1_;
    double t_ssp_;
    double t_dsp2_;

    int current_step_num_;
    int total_step_num_;

    // BIPED WALKING CONTROL
    void updateInitialState();
    void getRobotState();
    void walkingStateMachine();
    void calculateFootStepTotal();
    void supportToFloatPattern();
    void floatToSupportFootstep();
    void updateNextStepTime();

    void getZmpTrajectory();
    void addZmpOffset();
    void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void onestepZmp(unsigned int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py);

    void getComTrajectory(); 
    void previewcontroller(double dt, int NL, int tick, 
                           Eigen::Vector3d &x_k, Eigen::Vector3d &y_k, double &UX, double &UY,
                           Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, 
                           Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C);
    void preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C);
    void getFootTrajectory(); 
    void getPelvTrajectory();
    void contactWrenchCalculator();
    void computeIkControl(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q);

    Eigen::MatrixXd foot_step_;
    Eigen::MatrixXd foot_step_support_frame_;
    Eigen::MatrixXd foot_step_support_frame_offset_;

    Eigen::Isometry3d pelv_support_start_;
    Eigen::Isometry3d pelv_support_init_;
    Eigen::Vector2d del_zmp;
    Eigen::Vector2d cp_desired_;
    Eigen::Vector2d cp_measured_;
    Eigen::Vector2d cp_measured_LPF;
    Eigen::Vector2d cp_measured_thread_;
    Eigen::Vector2d cp_measured_mpc_;
    Eigen::Vector3d cp_float_current_;

    Eigen::Vector3d com_desired_;

    Eigen::Vector3d com_support_init_;
    Eigen::Vector3d com_float_init_;
    Eigen::Vector3d com_float_current_;
    Eigen::Vector3d com_support_current_;
    Eigen::Vector3d com_support_current_dot_;
    Eigen::Vector3d com_support_current_LPF;
    Eigen::Vector3d com_float_current_LPF;
    Eigen::Vector3d com_support_current_prev;
    Eigen::Vector3d com_support_cp_;

    Eigen::Vector3d com_float_current_dot;
    Eigen::Vector3d com_float_current_dot_prev;
    Eigen::Vector3d com_float_current_dot_LPF;
    Eigen::Vector3d com_support_current_dot_LPF;

    Eigen::Vector3d pelv_rpy_current_mj_;
    Eigen::Vector3d rfoot_rpy_current_;
    Eigen::Vector3d lfoot_rpy_current_;
    Eigen::Isometry3d pelv_yaw_rot_current_from_global_mj_;
    Eigen::Isometry3d rfoot_roll_rot_;
    Eigen::Isometry3d lfoot_roll_rot_;
    Eigen::Isometry3d rfoot_pitch_rot_;
    Eigen::Isometry3d lfoot_pitch_rot_;

    Eigen::Isometry3d pelv_float_current_;
    Eigen::Isometry3d lfoot_float_current_;
    Eigen::Isometry3d rfoot_float_current_;
    Eigen::Isometry3d pelv_float_init_;
    Eigen::Isometry3d lfoot_float_init_;
    Eigen::Isometry3d rfoot_float_init_;

    Eigen::Isometry3d pelv_trajectory_support_; //local frame
    Eigen::Isometry3d pelv_trajectory_support_fast_; //local frame
    Eigen::Isometry3d pelv_trajectory_support_slow_; //local frame
    
    Eigen::Isometry3d rfoot_trajectory_support_;  //local frame
    Eigen::Isometry3d lfoot_trajectory_support_;
    Eigen::Isometry3d lfoot_trajectory_support_fast_;
    Eigen::Isometry3d lfoot_trajectory_support_slow_;

    Eigen::Vector3d rfoot_trajectory_euler_support_;
    Eigen::Vector3d lfoot_trajectory_euler_support_;

    Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame

    Eigen::Vector3d com_trajectory_float_;

    Eigen::Isometry3d lfoot_trajectory_float_;
    Eigen::Isometry3d lfoot_trajectory_float_fast_;
    Eigen::Isometry3d lfoot_trajectory_float_slow_;

    Eigen::Isometry3d rfoot_trajectory_float_;
    Eigen::Isometry3d rfoot_trajectory_float_fast_;
    Eigen::Isometry3d rfoot_trajectory_float_slow_;

    Eigen::Vector3d pelv_support_euler_init_;
    Eigen::Vector3d lfoot_support_euler_init_;
    Eigen::Vector3d rfoot_support_euler_init_;
    double wn = 0;

    double walking_end_flag = 0;
    
    Eigen::Isometry3d swingfoot_float_current_; 
    Eigen::Isometry3d supportfoot_float_current_; 

    Eigen::Isometry3d pelv_support_current_;
    Eigen::Isometry3d lfoot_support_current_;
    Eigen::Isometry3d rfoot_support_current_;

    Eigen::Isometry3d lfoot_support_init_;
    Eigen::Isometry3d rfoot_support_init_;
    
    Eigen::Vector6d supportfoot_support_init_offset_;
    Eigen::Vector6d supportfoot_float_init_;
    Eigen::Vector6d supportfoot_support_init_;
    Eigen::Vector6d swingfoot_float_init_;
    Eigen::Vector6d swingfoot_support_init_;
    
    Eigen::MatrixXd ref_zmp_;
    
    int first_current_step_flag_ = 0;
    int first_current_step_number_ = 0;

    double F_F_input_dot = 0;
    double F_F_input = 0;

    double F_T_L_x_input = 0;
    double F_T_L_x_input_dot = 0;
    double F_T_R_x_input = 0;
    double F_T_R_x_input_dot = 0;  

    double F_T_L_y_input = 0;
    double F_T_L_y_input_dot = 0;
    double F_T_R_y_input = 0;
    double F_T_R_y_input_dot = 0;
    
    double Tau_L_x_error_ = 0;
    double Tau_L_x_error_pre_ = 0;
    double Tau_L_x_error_dot_ = 0;    

    double Tau_L_y_error_ = 0;
    double Tau_L_y_error_pre_ = 0;
    double Tau_L_y_error_dot_ = 0;    

    double Tau_R_x_error_ = 0;
    double Tau_R_x_error_pre_ = 0;
    double Tau_R_x_error_dot_ = 0;    

    double Tau_R_y_error_ = 0;
    double Tau_R_y_error_pre_ = 0;
    double Tau_R_y_error_dot_ = 0;    

    double F_F_error_ = 0;
    double F_F_error_pre_ = 0;
    double F_F_error_dot_ = 0; 

    double P_angle_i = 0;
    double P_angle = 0;
    double P_angle_input_dot = 0;
    double P_angle_input = 0;
    double R_angle = 0;
    double R_angle_input_dot = 0;
    double R_angle_input = 0;
    double aa = 0; 
    double Y_angle_input = 0;

    Eigen::Vector6d l_ft_;
    Eigen::Vector6d r_ft_;
    Eigen::Vector6d l_ft_LPF;
    Eigen::Vector6d r_ft_LPF;
    Eigen::Vector2d zmp_measured_mj_;
    Eigen::Vector2d zmp_measured_LPF_;

    // PREVIEW CONTROL
    Eigen::Vector3d x_preview_;
    Eigen::Vector3d y_preview_;

    Eigen::Vector3d xs_preview_;
    Eigen::Vector3d ys_preview_;
    Eigen::Vector3d xd_preview_;
    Eigen::Vector3d yd_preview_; 

    Eigen::MatrixXd Gi_preview_;
    Eigen::MatrixXd Gx_preview_;
    Eigen::VectorXd Gd_preview_;
    Eigen::MatrixXd A_preview_;
    Eigen::VectorXd B_preview_;
    Eigen::MatrixXd C_preview_;
    double UX_preview_, UY_preview_;

    double del_t = 0.0005;
    double xi_preview_;
    double yi_preview_;
    double zc_preview_;
    double ZMP_X_REF_;
    double ZMP_Y_REF_;
    double ZMP_Y_REF_alpha_;
    double alpha_lpf_ = 0.0;

    double zmp_start_time_; 
    double zmp_modif_time_margin_;

    // CONTACT WRENCH CONTROL

    Eigen::MatrixXd modified_del_zmp_; 
    Eigen::MatrixXd m_del_zmp_x;
    Eigen::MatrixXd m_del_zmp_y;

    Eigen::VectorQd swing_wrench_torque;
    Eigen::VectorQd contact_wrench_torque;
    Eigen::Vector6d rfoot_contact_wrench;
    Eigen::Vector6d lfoot_contact_wrench;

    double kp_cp = 0.0;

private:
    Eigen::VectorQd ControlVal_;
    unsigned int walking_tick = 0;
    unsigned int initial_tick_ = 0;
    const double hz_ = 2000.0;
};
