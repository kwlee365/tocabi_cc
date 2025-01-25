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

#include <filesystem>
#include <mpc.h>
#include <sensor_msgs/Joy.h>

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();
    double getMpcFrequency() const;

    void computeSlow();
    void computeFast();
    void computeThread3();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);
    void pubDataSlowToThread3();
    void subDataSlowToThread3();
    void pubDataThread3ToSlow();
    void subDataThread3ToSlow();

    int is_mode6_start = 0;

    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;
    // ros::Subscriber joy_sub_;

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
    bool is_mode_7_fin = true;
    bool is_joystick_mode = false;
    bool is_support_foot_change = false;    
    bool is_lfoot_support = false;
    bool is_rfoot_support = false;
    bool is_dsp1 = false;
    bool is_ssp  = false;
    bool is_dsp2 = false;
    bool is_preview_ctrl_init = true;
    bool is_mpc_ctrl_init = true;
    bool is_mpc_update = false;

    std::atomic<bool> atb_grav_update_{false};
    std::atomic<bool> atb_mpc_update_{false};

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

    int current_step_num_, current_step_num_container, current_step_num_thread3, current_step_num_checker;
    int total_step_num_;

    int pattern_poly_order = 1;

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
    double computeInterpolationFunction(int poly_order, double t, double T);

    void getComTrajectory(); 
    void getComTrajectory_mpc();
    void getContactIndicatorReference(const int &mpc_local_time, const int &mpc_start_time, const int &mpc_local_l_or_r, const int &mpc_local_step_num_, const int &mpc_iter);
    void getContactPointReference(const int &mpc_local_time, const int &mpc_start_time, const int &mpc_local_l_or_r, const int &mpc_local_step_num_, const Eigen::Vector3d &mpc_lfoot_support_init_, const Eigen::Vector3d &mpc_rfoot_support_init_, const double &mpc_step_length_x_temp, const double &mpc_step_length_y_temp, const int &mpc_iter);

    void previewcontroller(double dt, int NL, int tick, 
                           Eigen::Vector3d &x_k, Eigen::Vector3d &y_k, double &UX, double &UY,
                           const Eigen::MatrixXd &Gi, const Eigen::VectorXd &Gd, const Eigen::MatrixXd &Gx, 
                           const Eigen::MatrixXd &A,  const Eigen::VectorXd &B,  const Eigen::MatrixXd &C,
                           int controller_hz, int main_hz, bool &is_ctrl_init);
    void preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C);
    void preview_Parameter_MPC(double dt, int NL, Eigen::MatrixXd &Gi, Eigen::VectorXd &Gd, Eigen::MatrixXd &Gx, Eigen::MatrixXd &A, Eigen::VectorXd &B, Eigen::MatrixXd &C);
    void getFootTrajectory(); 
    void getPelvTrajectory();
    void contactWrenchCalculator();
    void computeIkControl(const Eigen::Isometry3d &float_trunk_transform, const Eigen::Isometry3d &float_lleg_transform, const Eigen::Isometry3d &float_rleg_transform, Eigen::Vector12d &q_des);

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
    Eigen::Vector3d com_desired_dot_;

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

    Eigen::Vector3d pelv_rpy_current_;
    Eigen::Vector3d rfoot_rpy_current_;
    Eigen::Vector3d lfoot_rpy_current_;
    Eigen::Isometry3d pelv_yaw_rot_current_from_global_;
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
    Eigen::MatrixXd ref_zmp_container;
    Eigen::MatrixXd ref_zmp_thread3;

    Eigen::VectorXd dcm_x_ref;
    Eigen::VectorXd dcm_x_ref_container;
    Eigen::VectorXd dcm_x_ref_thread3; 
    Eigen::VectorXd dcm_y_ref;
    Eigen::VectorXd dcm_y_ref_container;
    Eigen::VectorXd dcm_y_ref_thread3; 

    Eigen::MatrixXd com_ref;
    Eigen::MatrixXd com_ref_container;
    Eigen::MatrixXd com_ref_thread3; 

    Eigen::MatrixXd com_dot_ref;
    Eigen::MatrixXd com_dot_ref_container;
    Eigen::MatrixXd com_dot_ref_thread3; 

    Eigen::MatrixXd body_euler_ref;
    Eigen::MatrixXd body_euler_ref_container;
    Eigen::MatrixXd body_euler_ref_thread3; 

    Eigen::VectorXd zx_ref;
    Eigen::VectorXd zy_ref;

    Eigen::VectorXd zx_preview;
    Eigen::VectorXd zx_preview_container;
    Eigen::VectorXd zx_preview_thread3;

    Eigen::VectorXd zy_preview;
    Eigen::VectorXd zy_preview_container;
    Eigen::VectorXd zy_preview_thread3;

    Eigen::VectorXd eta_l_ref;
    Eigen::VectorXd eta_l_ref_container;
    Eigen::VectorXd eta_l_ref_thread3;

    Eigen::VectorXd eta_r_ref;
    Eigen::VectorXd eta_r_ref_container;
    Eigen::VectorXd eta_r_ref_thread3;

    Eigen::MatrixXd lfoot_contact_point_ref;
    Eigen::MatrixXd lfoot_contact_point_ref_container;
    Eigen::MatrixXd lfoot_contact_point_ref_thread3;

    Eigen::MatrixXd rfoot_contact_point_ref;
    Eigen::MatrixXd rfoot_contact_point_ref_container;
    Eigen::MatrixXd rfoot_contact_point_ref_thread3;

    Eigen::MatrixXd lfoot_contact_wrench_ref;
    Eigen::MatrixXd lfoot_contact_wrench_ref_container;
    Eigen::MatrixXd lfoot_contact_wrench_ref_thread3;

    Eigen::MatrixXd rfoot_contact_wrench_ref;
    Eigen::MatrixXd rfoot_contact_wrench_ref_container;
    Eigen::MatrixXd rfoot_contact_wrench_ref_thread3;
    
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

    // MODEL PREDICTIVE CONTROL
    Eigen::Vector3d x_preview_mpc;
    Eigen::Vector3d y_preview_mpc;

    Eigen::MatrixXd Gi_preview_mpc;
    Eigen::MatrixXd Gx_preview_mpc;
    Eigen::VectorXd Gd_preview_mpc;
    Eigen::MatrixXd A_preview_mpc;
    Eigen::VectorXd B_preview_mpc;
    Eigen::MatrixXd C_preview_mpc;
    double UX_preview_mpc, UY_preview_mpc;

    double zmp_preview_x = 0.0;
    double zmp_preview_y = 0.0;

    unsigned int mpc_interpol_cnt_x = 0;
    unsigned int mpc_interpol_cnt_y = 0;

    double del_t = 0.0005;
    double ZMP_X_REF_;
    double ZMP_Y_REF_;
    double ZMP_Y_REF_alpha_;
    double alpha_lpf_ = 0.0;

    // CONTACT WRENCH CONTROL
    Eigen::VectorQd swing_wrench_torque;
    Eigen::VectorQd contact_wrench_torque;
    Eigen::Vector6d rfoot_contact_wrench;
    Eigen::Vector6d lfoot_contact_wrench;

    Eigen::Vector6d rfoot_contact_wrench_mpc;
    Eigen::Vector6d lfoot_contact_wrench_mpc;
    Eigen::Vector6d rfoot_contact_wrench_container;
    Eigen::Vector6d lfoot_contact_wrench_container;
    Eigen::Vector6d rfoot_contact_wrench_thread3;
    Eigen::Vector6d lfoot_contact_wrench_thread3;

    double kp_cp = 0.0;
    double zmp_offset = 0.0;

    int pred_footstep_num = 0;
    
    template <typename EigenType>
    void recordData(const int &file_number, const EigenType &data_);
    void initFile();

    constexpr static int NUM_PLOT{50};
	ofstream plot_files_[NUM_PLOT];
	const string string_plot_files_names_[NUM_PLOT]
	{"/home/kwan/catkin_ws/src/tocabi_cc/data/data1",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data2",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data3",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data4",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data5",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data6",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data7",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data8",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data9",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data10",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data11",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data12",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data13",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data14",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data15",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data16",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data17",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data18",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data19",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data20",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data21",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data22",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data23",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data24",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data25",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data26",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data27",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data28",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data29",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data30",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data31",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data32",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data33",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data34",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data35",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data36",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data37",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data38",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data39",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data40",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data41",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data42",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data43",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data44",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data45",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data46",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data47",
     "/home/kwan/catkin_ws/src/tocabi_cc/data/data48",
    };

private:
    Eigen::VectorQd ControlVal_;
    unsigned int walking_tick = 0;
    unsigned int walking_tick_container = 0;
    unsigned int walking_tick_thread3 = 0;

    unsigned int zmp_start_time_ = 0; 
    unsigned int zmp_start_time_container = 0; 
    unsigned int zmp_start_time_thread3 = 0; 

    unsigned int initial_tick_ = 0;
    const double hz_ = 2000.0;

    const double mpc_freq = 50.0;
    const double mpc_N  = 10.0;
};
