#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <list>
#include <iomanip> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "dyn_wbc.h"
#include "utils.h"

enum class TestMotionType {
    None,
    PelvHand,
    PelvHandJoy,
    Taichi,
    Walking
};

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void xBoxJoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::Subscriber joy_sub_;
    ros::Subscriber xbox_joy_sub_;

    double target_vel_x_ = 0.0;
    double target_vel_y_ = 0.0;
    double target_vel_yaw_ = 0.0;

    void loadParams();

    //--- Thread
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    //--- State Machine
    bool is_mode_6_init = true;
    bool is_mode_7_init = true;
    bool is_mode_7_working = true;
    bool is_torque_desired_init = true;
    bool is_derivative_init = true;
    bool is_torque_transition = false;
    bool is_left_contact_transition = false;
    bool is_right_contact_transition = false;
    

    //--- Robot Model
    RigidBodyDynamics::Model model_;  
    DynWBC dyn_wbc_;  
    std::vector<std::vector<TaskInfo>> task_hierarchy;
    ContactIndicator contact_mode_;
    unsigned int contact_dim = 12;

    RobotData &rd_;
    RobotData rd_cc_;

    Eigen::VectorXd Kp; Eigen::MatrixXd Kp_diag;
    Eigen::VectorXd Kd; Eigen::MatrixXd Kd_diag;
    Eigen::VectorXd Kp_virtual; Eigen::MatrixVVd Kp_virtual_diag;
    Eigen::VectorXd Kd_virtual; Eigen::MatrixVVd Kd_virtual_diag;
    std::map<std::string, Eigen::VectorXd> W_task;
    Eigen::VectorQd W_energy;     
    Eigen::VectorXd W_contact;     
    Eigen::VectorQd W_torque_prev;
    Eigen::VectorQd joint_pos_limit_l_;
    Eigen::VectorQd joint_pos_limit_h_;
    Eigen::VectorQd joint_vel_limit_l_;
    Eigen::VectorQd joint_vel_limit_h_;

    //--- Robot State
    void stateManager();
    void contactStateManager();
    void taskStateManager();
    void saveInitialState();
    
    std::string base_link_name  = "Pelvis_Link";
    std::string chest_link_name = "Upperbody_Link";
    std::string lfoot_link_name = "L_Foot_Link";
    std::string rfoot_link_name = "R_Foot_Link";
    std::string lhand_link_name = "L_Wrist2_Link";
    std::string rhand_link_name = "R_Wrist2_Link";
    std::string head_link_name  = "Head_Link";
    std::string com_name        = "COM_id";

    std::map<std::string, int> link_index_map = {   // The links should be ordered according to the inverse kinematics hierarchy
        {base_link_name, 0},
        {chest_link_name, 3},
        {lfoot_link_name, 9},
        {rfoot_link_name, 15},
        {lhand_link_name, 23},
        {rhand_link_name, 31},
        {head_link_name, 33},
        {com_name, 34}
    };

    // Robot state w.r.t. global frame
    std::map<std::string, Eigen::Matrix3Vd> Jac_v;
    std::map<std::string, Eigen::Matrix3Vd> Jac_w;
    Eigen::MatrixXd contact_Jac;

    std::map<std::string, Eigen::Vector3d> ee_pos;
    std::map<std::string, Eigen::Matrix3d> ee_rot;
    std::map<std::string, Eigen::Vector3d> ee_v;
    std::map<std::string, Eigen::Vector3d> ee_w;
    std::map<std::string, Eigen::Vector3d>  init_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  init_ee_rot;
    std::map<std::string, Eigen::Vector3d>  init_ee_v;
    std::map<std::string, Eigen::Vector3d>  init_ee_w;

    // Robot state w.r.t. base frame
    std::map<std::string, Eigen::Matrix6Vd> base_Jac;
    std::map<std::string, Eigen::Matrix6Vd> base_Jac_dot;

    Eigen::MatrixXd base_contact_Jac;
    Eigen::MatrixXd base_contact_Jac_dot;
    Eigen::MatrixXd base_contact_lambda;
    Eigen::MatrixXd base_contact_Jac_inv_T;
    Eigen::MatrixVVd base_contact_N;

    std::map<std::string, Eigen::MatrixXd> base_task_lambda;
    std::map<std::string, Eigen::MatrixXd> base_task_Jac_inv_T;
    std::map<std::string, Eigen::MatrixXd> base_task_Jac_inv_T_S_T;

    std::map<std::string, Eigen::Matrix3Vd> base_Jac_v;
    std::map<std::string, Eigen::Matrix3Vd> base_Jac_w;
    std::map<std::string, Eigen::Matrix3Vd> base_Jac_v_prev;
    std::map<std::string, Eigen::Matrix3Vd> base_Jac_w_prev;
    std::map<std::string, Eigen::Matrix3Vd> base_Jac_v_dot;
    std::map<std::string, Eigen::Matrix3Vd> base_Jac_w_dot;

    std::map<std::string, Eigen::Vector3d>  base_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  base_ee_rot;
    std::map<std::string, Eigen::Vector3d>  base_ee_v;
    std::map<std::string, Eigen::Vector3d>  base_ee_w;
    std::map<std::string, Eigen::Vector3d>  init_base_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  init_base_ee_rot;
    std::map<std::string, Eigen::Vector3d>  init_base_ee_v;
    std::map<std::string, Eigen::Vector3d>  init_base_ee_w;

    // Robot state w.r.t. support frame
    std::map<std::string, Eigen::Vector3d>  support_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  support_ee_rot;
    std::map<std::string, Eigen::Vector3d>  support_ee_v;
    std::map<std::string, Eigen::Vector3d>  support_ee_w;
    std::map<std::string, Eigen::Vector3d>  init_support_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  init_support_ee_rot;
    std::map<std::string, Eigen::Vector3d>  init_support_ee_v;
    std::map<std::string, Eigen::Vector3d>  init_support_ee_w;

    Eigen::MatrixXd M_temp_;
    Eigen::VectorXd G_temp_;
    Eigen::MatrixVVd M_;
    Eigen::MatrixVVd M_inv_;
    Eigen::VectorVQd G_;
    //---

    //--- Initial Values
    Eigen::VectorQd q_init_;
    Eigen::VectorQd q_init_des;
    void moveInitialPose();

    //--- Desired Variables
    std::map<std::string, Eigen::Matrix3d> R_desired;
    std::map<std::string, Eigen::Vector3d> x_desired, dx_desired, ddx_desired, w_desired, dw_desired;
    std::map<std::string, Eigen::Matrix3d> support_R_desired;
    std::map<std::string, Eigen::Vector3d> support_x_desired, support_dx_desired, support_ddx_desired, support_w_desired, support_dw_desired;
    std::map<std::string, Eigen::Vector6d> wrench_desired;
    std::map<std::string, Eigen::Vector3d> task_Kp; 
    std::map<std::string, Eigen::Vector3d> task_Kv; 
    Eigen::VectorVQd q_, qdot_;
    Eigen::VectorVQd q_des, dq_des, qdot_des, qddot_des;
    Eigen::VectorQd torque_transition;

    //--- Test Function
    void movePelvPose(double traj_time, double pelv_dist);
    void moveHandPose(double traj_time, double hand_dist);
    void movePelvHandPose(double traj_time, double pelv_dist, double hand_dist);
    void movePelvPoseJoy(const double& vx, const double& vy, const double& wz);
    void movePelvHandPoseJoy(const double& target_vel_x_, const double& target_vel_y_, const double& target_vel_yaw_, const double& traj_time, const double& hand_dist);
    void moveTaichiMotion(const double& traj_time, const double& pelv_dist, const double& hand_dist, const double& foot_height);
    void runTestMotion(const double& traj_time, const double& pelv_dist, const double& hand_dist, const double& foot_height, const double& swing_duration);
    TestMotionType motion_mode_ = TestMotionType::None;


private:
    Eigen::VectorQd ControlVal_;
    double hz_ = 2000;

    const double NM2CNT[MODEL_DOF] =
        {  
            3.0,  //left Leg
            4.3,
            3.8,
            3.46,
            4.5,
            6.0,
            
            3.0,  //right Leg
            4.3,
            3.8,
            3.46,
            4.5,
            6.0,
            
            3.3,  //Waist
            3.3,            
            3.3,  //upperbody
            
            15.5, //shoulder2
            15.5, //shoulder1
            15.5, //shoulder2
            15.5, //arm
            42.0, //Elbow
            42.0, //Forearm 
            95.0, //wrist
            95.0,
            
            95.0, //head
            95.0,
            
            15.5, //shoulder2
            15.5, //shoulder1
            15.5, //shoulder2
            15.5, //arm
            42.0, //Elbow
            42.0, //Forearm 
            95.0, //wrist
            95.0
        };

        // Damping values for each joint based on its speed reducer type
        const double jointDamping[MODEL_DOF] = {
            0.0248, // HipYaw (shg20_100_2so)
            0.0248, // HipRoll (shg20_100_2so)
            0.0248, // HipPitch (shg20_100_2so)
            0.0248, // KneePitch (shg20_100_2so)
            0.0248, // AnklePitch (shg20_100_2so)
            0.0161, // AnkleRoll (shd20_100_2sh)

            0.0248, // HipYaw (shg20_100_2so)
            0.0248, // HipRoll (shg20_100_2so)
            0.0248, // HipPitch (shg20_100_2so)
            0.0248, // KneePitch (shg20_100_2so)
            0.0248, // AnklePitch (shg20_100_2so)
            0.0161, // AnkleRoll (shd20_100_2sh)

            0.0417, // WaistYaw (shg25_100_2so)
            0.0417, // WaistPitch (shg25_100_2so)
            0.0417, // WaistRoll (shg25_100_2so)

            0.0148, // Shoulder1 (shg17_100_2so)
            0.0148, // Shoulder2 (shg17_100_2so)
            0.0148, // Shoulder3 (shg17_100_2so)
            0.0148, // Armlink (shg17_100_2so)
            0.0047, // Elbow (shg14_100_2so)
            0.0047, // ForeArm (shg14_100_2so)
            0.0029, // Wrist1 (csf_11_100_2xh_f)
            0.0029, // Wrist2 (csf_11_100_2xh_f)

            0.0029, // Head1 (csf_11_100_2xh_f)
            0.0029,  // Head2 (csf_11_100_2xh_f)

            0.0148, // Shoulder1 (shg17_100_2so)
            0.0148, // Shoulder2 (shg17_100_2so)
            0.0148, // Shoulder3 (shg17_100_2so)
            0.0148, // Armlink (shg17_100_2so)
            0.0047, // Elbow (shg14_100_2so)
            0.0047, // ForeArm (shg14_100_2so)
            0.0029, // Wrist1 (csf_11_100_2xh_f)
            0.0029 // Wrist2 (csf_11_100_2xh_f)
        };

        // Friction loss values for each joint based on its speed reducer type
        const double jointFrictionLoss[MODEL_DOF] = {
            9.9,  // HipYaw (shg20_100_2so)
            9.9,  // HipRoll (shg20_100_2so)
            9.9,  // HipPitch (shg20_100_2so)
            9.9,  // KneePitch (shg20_100_2so)
            9.9,  // AnklePitch (shg20_100_2so)
            22.0, // AnkleRoll (shd20_100_2sh)

            14.0, // WaistYaw (shg25_100_2so)
            14.0, // WaistPitch (shg25_100_2so)
            14.0, // WaistRoll (shg25_100_2so)

            6.5,  // Shoulder1 (shg17_100_2so)
            6.5,  // Shoulder2 (shg17_100_2so)
            6.5,  // Shoulder3 (shg17_100_2so)
            6.5,  // Armlink (shg17_100_2so)
            3.7,  // Elbow (shg14_100_2so)
            3.7,  // ForeArm (shg14_100_2so)
            1.5,  // Wrist1 (csf_11_100_2xh_f)
            1.5,  // Wrist2 (csf_11_100_2xh_f)

            1.5,  // Head1 (csf_11_100_2xh_f)
            1.5   // Head2 (csf_11_100_2xh_f)
        };
        
    const double gear_ratio = 100.0;
};