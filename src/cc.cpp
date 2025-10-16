#include "cc.h"

using namespace TOCABI;

ofstream dataCC1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC1.txt");
ofstream dataCC2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC2.txt");

CustomController::CustomController(RobotData &rd) : rd_(rd), cm_(rd), tm_(rd), kin_wbc_(rd), dyn_wbc_(rd)
{
    //--- ROS Node Handle
    nh_cc_.setCallbackQueue(&queue_cc_);
    ControlVal_.setZero();

    //--- Class Initialization
    tm_.setControlFrequency(hz_);
    dyn_wbc_.setFrictionCoefficient(0.8);
    dyn_wbc_.setFootDimension(0.3, 0.16);

    //--- Joy Callback
    xbox_joy_sub_ = nh_cc_.subscribe<sensor_msgs::Joy>("/joy", 10, &CustomController::xBoxJoyCallback, this);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}
/* ========================================================================================
 * Real-time Control Thread
 * Control Frequency : 2 kHz
 * 
 * Control Manager (cm_) : 
 *      - Update Robot Model,
 *      - Manage Robot State, Contact State, Dynamics
 * 
 * Task Manager (tm_) : 
 *      - Update Task trajectory (PelvHand, Taichi, Walking)
 * 
 * Kinematic WBC (kin_wbc_) : 
 *      - Compute desired joint velocity using Hierarchical Inverse Kinematics, 
 *      - Safety Joint Velocity Filter
 * ========================================================================================
 */

void CustomController::computeSlow()
{
    queue_cc_.callAvailable(ros::WallDuration());
    
    if (rd_.tc_.mode == 6)
    {   
        CustomControllerInit();
        moveInitialPose();
    }
    else if (rd_.tc_.mode == 7)
    {
        cm_.update();

        tm_.runTestMotion(motion_mode_); 

        kin_wbc_.computeTaskSpaceKinematicWBC();

        dyn_wbc_.computeDynamicWBC();
        dyn_wbc_.computeTotalTorqueCommand();

        dataCC1 << rd_.LF_FT.transpose() << " " << rd_.RF_FT.transpose() << std::endl;
        dataCC2 << rd_.torque_desired.transpose() << std::endl;

    }
    else
    {
        rd_.torque_desired = (rd_.Kp_diag * (rd_.q_desired - rd_.q_)) - (rd_.Kd_diag * rd_.q_dot_);
    }
}

void CustomController::computeFast()
{

}

void CustomController::computePlanner()
{

}

void CustomController::CustomControllerInit()
{
    static bool is_cc_init = true;

    if (is_cc_init == true)
    {
        loadParams();

        q_init_ = rd_.q_;
        WBC::SetContact(rd_, true, true);
        
        cout << "========== CUSTOMCONTROLLER IS NOW INITIALIZED ==========" << endl;
        cout << "TIME: "<< rd_.control_time_ << endl; 

        is_cc_init = false;
    }
}

void CustomController::moveInitialPose()
{
    static int initial_tick = 0;

    q_init_des; q_init_des.setZero();
    q_init_des = q_init_;

    if(motion_mode_ == TaskMotionType::Walking)
    {
        q_init_des(0) = 0.0; 
        q_init_des(1) = 0.0; 
        q_init_des(2) = -0.24; 
        q_init_des(3) = 0.6; 
        q_init_des(4) = -0.36; 
        q_init_des(5) = 0.0; 

        q_init_des(6)  = 0.0; 
        q_init_des(7)  = 0.0; 
        q_init_des(8)  = -0.24; 
        q_init_des(9)  = 0.6; 
        q_init_des(10) = -0.36; 
        q_init_des(11) = 0.0;

        q_init_des(15) = + 15.0 * DEG2RAD; 
        q_init_des(16) = + 10.0 * DEG2RAD; 
        q_init_des(17) = + 80.0 * DEG2RAD; 
        q_init_des(18) = - 70.0 * DEG2RAD; 
        q_init_des(19) = - 45.0 * DEG2RAD; 
        q_init_des(21) =   0.0 * DEG2RAD; 

        q_init_des(25) = - 15.0 * DEG2RAD; 
        q_init_des(26) = - 10.0 * DEG2RAD;            
        q_init_des(27) = - 80.0 * DEG2RAD;  
        q_init_des(28) = + 70.0 * DEG2RAD; 
        q_init_des(29) = + 45.0 * DEG2RAD;       
        q_init_des(31) = - 0.0 * DEG2RAD; 
    }
    else
    {
        q_init_des(15) = 0.0;
        q_init_des(16) = -0.3;
        q_init_des(17) = 1.57;
        q_init_des(18) = -1.2;
        q_init_des(19) = -1.57; // elbow
        q_init_des(20) = 1.5;
        q_init_des(21) = 0.4;
        q_init_des(22) = -0.2;

        q_init_des(23) = 0.0; // yaw
        q_init_des(24) = 0.0; // pitch

        q_init_des(25) = 0.0;
        q_init_des(26) = 0.3;
        q_init_des(27) = -1.57;
        q_init_des(28) = 1.2;
        q_init_des(29) = 1.57; // elbow
        q_init_des(30) = -1.5;
        q_init_des(31) = -0.4;
        q_init_des(32) = 0.2;
    }
    
    rd_.q_desired = DyrosMath::cubicVector<MODEL_DOF>(initial_tick, 0, 2.0 * hz_, q_init_, q_init_des, Eigen::VectorQd::Zero(), Eigen::VectorQd::Zero()); 
    rd_.torque_desired = (rd_.Kp_diag * (rd_.q_desired - rd_.q_)) - (rd_.Kd_diag * rd_.q_dot_);

    initial_tick++;
}

//--- Joy Utils
void CustomController::xBoxJoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double vel_threshold = 0.1;

    target_vel_x_   = DyrosMath::minmax_cut(joy->axes[1] * vel_threshold, -vel_threshold, vel_threshold);
    target_vel_y_   = DyrosMath::minmax_cut(joy->axes[0] * vel_threshold, -vel_threshold, vel_threshold);
    target_vel_yaw_ = DyrosMath::minmax_cut(joy->axes[3] * vel_threshold, -vel_threshold, vel_threshold);
}

//--- Parameter Loader

void CustomController::loadParams()
{
    Kp.setZero(MODEL_DOF); Kd.setZero(MODEL_DOF);
    Kp_virtual.setZero(MODEL_DOF_VIRTUAL); Kd_virtual.setZero(MODEL_DOF_VIRTUAL);
    
    
    std::vector<double> kp_vec, kd_vec;
    std::vector<double> kp_dyn_vec, kd_dyn_vec;
    std::vector<double> pos_low_deg, pos_high_deg;
    std::vector<double> vel_low, vel_high;

    nh_cc_.getParam("/tocabi_controller/joint_gains/Kp", kp_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Kd", kd_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Kp_dyn", kp_dyn_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Kd_dyn", kd_dyn_vec);
    nh_cc_.getParam("/tocabi_controller/joint_limits/pos_low_deg", pos_low_deg);
    nh_cc_.getParam("/tocabi_controller/joint_limits/pos_high_deg", pos_high_deg);
    nh_cc_.getParam("/tocabi_controller/joint_limits/vel_low", vel_low);
    nh_cc_.getParam("/tocabi_controller/joint_limits/vel_high", vel_high);

    // Check Vector Dimension
    if (kp_vec.size() != MODEL_DOF)
        ROS_ERROR("Kp vector size mismatch: got %lu, expected %d", kp_vec.size(), MODEL_DOF);
    assert(kp_vec.size() == MODEL_DOF);

    if (kd_vec.size() != MODEL_DOF)
        ROS_ERROR("Kd vector size mismatch: got %lu, expected %d", kd_vec.size(), MODEL_DOF);
    assert(kd_vec.size() == MODEL_DOF);

    if (kp_dyn_vec.size() != MODEL_DOF_VIRTUAL)
        ROS_ERROR("Kp_dyn vector size mismatch: got %lu, expected %d", kp_dyn_vec.size(), MODEL_DOF_VIRTUAL);
    assert(kp_dyn_vec.size() == MODEL_DOF_VIRTUAL);

    if (kd_dyn_vec.size() != MODEL_DOF_VIRTUAL)
        ROS_ERROR("Kd_dyn vector size mismatch: got %lu, expected %d", kd_dyn_vec.size(), MODEL_DOF_VIRTUAL);
    assert(kd_dyn_vec.size() == MODEL_DOF_VIRTUAL);

    if (pos_low_deg.size() != MODEL_DOF)
        ROS_ERROR("Joint position lower limit vector size mismatch: got %lu, expected %d", pos_low_deg.size(), MODEL_DOF);
    assert(pos_low_deg.size() == MODEL_DOF);

    if (pos_high_deg.size() != MODEL_DOF)
        ROS_ERROR("Joint position upper limit vector size mismatch: got %lu, expected %d", pos_high_deg.size(), MODEL_DOF);
    assert(pos_high_deg.size() == MODEL_DOF);

    if (vel_low.size() != MODEL_DOF)
        ROS_ERROR("Joint velocity lower limit vector size mismatch: got %lu, expected %d", pos_low_deg.size(), MODEL_DOF);
    assert(vel_low.size() == MODEL_DOF);

    if (vel_high.size() != MODEL_DOF)
        ROS_ERROR("Joint velocity upper limit vector size mismatch: got %lu, expected %d", pos_high_deg.size(), MODEL_DOF);
    assert(vel_high.size() == MODEL_DOF);

    // Assign each vector into Eigen Vec or Mat
    for (int i = 0; i < MODEL_DOF; ++i)
    {
        Kp(i) = kp_vec[i];
        Kd(i) = kd_vec[i];
    }
    for (int i = 0; i < MODEL_DOF_VIRTUAL; ++i)
    {
        Kp_virtual(i) = kp_dyn_vec[i];
        Kd_virtual(i) = kd_dyn_vec[i] * 1.5;
    }

    rd_.Kp_virtual_diag = Kp_virtual.asDiagonal();
    rd_.Kd_virtual_diag = Kd_virtual.asDiagonal();
    rd_.Kp_diag = Kp.asDiagonal();
    rd_.Kd_diag = Kd.asDiagonal();

    // Position Limits (convert deg to rad)
    for (int i = 0; i < MODEL_DOF; ++i)
    {
        rd_.q_pos_l_lim(i) = pos_low_deg[i] * DEG2RAD;
        rd_.q_pos_h_lim(i) = pos_high_deg[i] * DEG2RAD;
        rd_.q_vel_l_lim(i) = vel_low[i];
        rd_.q_vel_h_lim(i) = vel_high[i];
    }

    // Motion mode
    int motion_mode_idx = 0;
    nh_cc_.getParam("/tocabi_controller/motion_mode", motion_mode_idx);
    if      (motion_mode_idx == 0){ motion_mode_ = TaskMotionType::None; }
    else if (motion_mode_idx == 1){ motion_mode_ = TaskMotionType::PelvHand;}
    else if (motion_mode_idx == 2){ motion_mode_ = TaskMotionType::Taichi; }
    else if (motion_mode_idx == 3){ motion_mode_ = TaskMotionType::Walking; }
    else {
        ROS_ERROR("Motion mode idx error: got %d", motion_mode_idx);
        assert(motion_mode_idx == 0 || motion_mode_idx == 1 || motion_mode_idx == 2 || motion_mode_idx == 3);
    }
    const char *mode_name =
        (motion_mode_ == TaskMotionType::None) ? "None" 
      : (motion_mode_ == TaskMotionType::PelvHand) ? "PelvHand"
      : (motion_mode_ == TaskMotionType::Taichi)   ? "Taichi"
      : (motion_mode_ == TaskMotionType::Walking)  ? "Walking"
      : "Unknown";

    std::cout << "=====================================" << std::endl;
    std::cout << "===== Motion Mode : " << mode_name << " =====" << std::endl;
    std::cout << "=====================================" << std::endl;

    //--- Task Parameter
    double traj_time_, pelv_dist_, hand_dist_, step_length_, foot_height_, step_duration_;
    nh_cc_.getParam("/tocabi_controller/task_param/traj_time", traj_time_);
    nh_cc_.getParam("/tocabi_controller/task_param/pelv_dist", pelv_dist_);
    nh_cc_.getParam("/tocabi_controller/task_param/hand_dist", hand_dist_);
    nh_cc_.getParam("/tocabi_controller/task_param/step_length", step_length_);
    nh_cc_.getParam("/tocabi_controller/task_param/foot_height", foot_height_);
    nh_cc_.getParam("/tocabi_controller/task_param/step_duration", step_duration_);

    tm_.setTrajectoryDuration(traj_time_);
    tm_.setPelvisDistance(pelv_dist_);
    tm_.setHandDistance(hand_dist_);
    tm_.setStepStride(step_length_);
    tm_.setFootHeight(foot_height_);
    tm_.setStepDuration(step_duration_);

    std::cout << "=====================================" << std::endl;
    std::cout << "========== Task Parameters ========== " << std::endl;
    std::cout << "Trajectory Time : " << traj_time_ << " sec" << std::endl;
    std::cout << "Pelvis Distance : " << pelv_dist_ << " m" << std::endl;
    std::cout << "Hand Distance : " << hand_dist_ << " m" << std::endl;
    std::cout << "Step Length : " << step_length_ << " m" << std::endl;
    std::cout << "Foot Height : " << foot_height_ << " m" << std::endl;
    std::cout << "Step Duration : " << step_duration_ << " sec" << std::endl;
    std::cout << "=====================================" << std::endl;

    //--- Whole-body Inverse Dynamics
    double W_qddot, W_cwr, W_energy;
    nh_cc_.getParam("/tocabi_controller/wbid/W_qddot", W_qddot);
    nh_cc_.getParam("/tocabi_controller/wbid/W_cwr", W_cwr);
    nh_cc_.getParam("/tocabi_controller/wbid/W_energy", W_energy);

    dyn_wbc_.setJointTrackingWeight(W_qddot);
    dyn_wbc_.setContactWrenchRegularizationWeight(W_cwr);
    dyn_wbc_.setAccelEnergyMinimizationWeight(W_energy);

    std::cout << "=====================================" << std::endl;
    std::cout << "========== WBID Parameters ========== " << std::endl;
    std::cout << "W_qddot : " << W_qddot  << std::endl;
    std::cout << "W_cwr : " << W_cwr  << std::endl;
    std::cout << "W_energy : " << W_energy  << std::endl;
}
