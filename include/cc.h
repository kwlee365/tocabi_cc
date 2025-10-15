#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <list>
#include <iomanip> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"


#include "control_manager.h"
#include "task_manager.h"
#include "kin_wbc.h"
#include "dyn_wbc.h"
#include "utils.h"

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

    Eigen::Vector3d v_cmd;
    Eigen::Vector3d w_cmd;

    void loadParams();

    //--- Thread
    void computeSlow();
    void computeFast();
    void computePlanner();

    //--- Robot Model
    RigidBodyDynamics::Model model_;  
    ControlManager cm_;
    TaskManager tm_;
    KinWBC kin_wbc_;  
    DynWBC dyn_wbc_;  
    
    std::vector<std::vector<TaskInfo>> task_hierarchy;
    std::set<std::string> task_names;
    ContactIndicator contact_mode_;
    unsigned int contact_dim = 12;

    RobotData &rd_;
    RobotData rd_cc_;

    Eigen::VectorXd Kp;  Eigen::VectorXd Kd; 
    Eigen::VectorXd Kp_virtual; Eigen::VectorXd Kd_virtual; 
    Eigen::VectorQd joint_pos_limit_l_;
    Eigen::VectorQd joint_pos_limit_h_;
    Eigen::VectorQd joint_vel_limit_l_;
    Eigen::VectorQd joint_vel_limit_h_;

    //--- Robot State
    void stateManager();
    void contactStateManager();
    void saveInitialState();
    double getSignedDistanceFunction(LinkData &linkA_, LinkData &linkB_, Eigen::MatrixXd &J_AB);
    

    //--- Initial Values
    Eigen::VectorQd q_init_;
    Eigen::VectorQd q_init_des;
    void CustomControllerInit();
    void moveInitialPose();

    //--- Test Function
    void movePelvPose(double traj_time, double pelv_dist);
    void moveHandPose(double traj_time, double hand_dist);
    void movePelvHandPose(double traj_time, double pelv_dist, double hand_dist);
    void moveTaichiMotion(const double& traj_time, const double& pelv_dist, const double& hand_dist, const double& foot_height);
    void bipedalWalkingController(const double& step_duration, const double& foot_height, const double& vx, const double& vy, const double& wz);
    void runTestMotion(const double& traj_time, const double& pelv_dist, const double& hand_dist, const double& foot_height, const double& step_duration);
    TaskMotionType motion_mode_ = TaskMotionType::None;


private:
    Eigen::VectorQd ControlVal_;
    double hz_ = 2000;
};