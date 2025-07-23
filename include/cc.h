#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <string>
#include <map>
#include <list>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "kin_wbc.h"
#include "dyn_wbc.h"
#include "utils.h"

enum class TestMotionType {
    None,
    Pelv,
    Hand,
    PelvHand,
    PelvJoy
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
    bool is_derivative_init = true;
    bool is_mode_temp_init = true;
    

    //--- Robot Model
    RigidBodyDynamics::Model model_;  
    KinWBC kin_wbc_;  
    DynWBC dyn_wbc_;  
    std::vector<std::vector<TaskInfo>> task_hierarchy;

    RobotData &rd_;
    RobotData rd_cc_;

    Eigen::VectorXd Kp; Eigen::MatrixXd Kp_diag;
    Eigen::VectorXd Kd; Eigen::MatrixXd Kd_diag;
    Eigen::VectorQd joint_pos_limit_l_;
    Eigen::VectorQd joint_pos_limit_h_;
    Eigen::VectorQd joint_vel_limit_l_;
    Eigen::VectorQd joint_vel_limit_h_;

    //--- Robot State
    void stateManager();
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
    std::map<std::string, Eigen::Vector3d> ee_pos;
    std::map<std::string, Eigen::Matrix3d> ee_rot;
    std::map<std::string, Eigen::Vector3d> ee_v;
    std::map<std::string, Eigen::Vector3d> ee_w;
    Eigen::MatrixXd contact_Jac;

    // Robot state w.r.t. base frame
    std::map<std::string, Eigen::Matrix3Vd> base_Jac_v;
    std::map<std::string, Eigen::Matrix3Vd> base_Jac_w;
    std::map<std::string, Eigen::Matrix3Vd> base_Jac_v_pre;
    std::map<std::string, Eigen::Matrix3Vd> base_Jac_w_pre;
    std::map<std::string, Eigen::Matrix3Vd> base_Jacdot_v;
    std::map<std::string, Eigen::Matrix3Vd> base_Jacdot_w;
    std::map<std::string, Eigen::Vector3d>  base_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  base_ee_rot;
    std::map<std::string, Eigen::Vector3d>  base_ee_v;
    std::map<std::string, Eigen::Vector3d>  base_ee_w;
    Eigen::MatrixXd base_contact_Jac;

    std::map<std::string, Eigen::Vector3d>  init_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  init_ee_rot;
    std::map<std::string, Eigen::Vector3d>  init_ee_v;
    std::map<std::string, Eigen::Vector3d>  init_ee_w;

    std::map<std::string, Eigen::Vector3d>  init_base_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  init_base_ee_rot;
    std::map<std::string, Eigen::Vector3d>  init_base_ee_v;
    std::map<std::string, Eigen::Vector3d>  init_base_ee_w;

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
    std::map<std::string, Eigen::Vector3d> task_Kp; 
    std::map<std::string, Eigen::Vector3d> task_Kv; 
    Eigen::VectorQd q_des;


    //--- Test Function
    void movePelvPose(double traj_time, double pelv_dist);
    void moveHandPose(double traj_time, double hand_dist);
    void movePelvHandPose(double traj_time, double pelv_dist, double hand_dist);
    void movePelvPoseJoy(const double& vx, const double& vy, const double& wz);
    void runTestMotion(double traj_time, double pelv_dist, double hand_dist);
    TestMotionType motion_mode_ = TestMotionType::None;
private:
    Eigen::VectorQd ControlVal_;
    double hz_ = 2000;
};