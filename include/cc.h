#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "kin_wbc.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <string>
#include <map>
#include <list>

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;

    void loadParams();

    //--- Thread
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    //--- State Machine
    bool is_mode_6_init = true;
    bool is_mode_7_init = true;
    bool is_mode_temp_init = true;

    //--- Robot Model
    RigidBodyDynamics::Model model_;  
    KinWBC kin_wbc_;  
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

    std::map<std::string, int> link_index_map = {   // The links should be ordered according to the inverse kinematics hierarchy
        {"Pelvis_Link", 0},
        {"Upperbody_Link", 3},
        {"L_Foot_Link", 9},
        {"R_Foot_Link", 15},
        {"L_Wrist2_Link", 23},
        {"R_Wrist2_Link", 31},
        {"Head_Link", 33},
        {"COM_id", 34}
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
    std::map<std::string, Eigen::Vector3d>  base_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  base_ee_rot;
    std::map<std::string, Eigen::Vector3d>  base_ee_v;
    std::map<std::string, Eigen::Vector3d>  base_ee_w;
    Eigen::MatrixXd base_contact_Jac;

    std::map<std::string, Eigen::Vector3d>  init_base_ee_pos;
    std::map<std::string, Eigen::Matrix3d>  init_base_ee_rot;
    std::map<std::string, Eigen::Vector3d>  init_base_ee_v;
    std::map<std::string, Eigen::Vector3d>  init_base_ee_w;

    Eigen::MatrixXd M_temp_;
    Eigen::VectorXd G_temp_;
    Eigen::MatrixVVd M_;
    Eigen::VectorVQd G_;
    //---

    //--- Initial Values
    Eigen::VectorQd q_init_;
    Eigen::VectorQd q_init_des;
    void moveInitialPose();

private:
    Eigen::VectorQd ControlVal_;
    double hz_ = 2000;
};