#include "cc.h"
using namespace TOCABI;

ofstream data1("/home/kwan/catkin_ws/src/tocabi_cc/data/data1.txt");
ofstream data2("/home/kwan/catkin_ws/src/tocabi_cc/data/data2.txt");
ofstream data3("/home/kwan/catkin_ws/src/tocabi_cc/data/data3.txt");
ofstream data4("/home/kwan/catkin_ws/src/tocabi_cc/data/data4.txt");
ofstream data5("/home/kwan/catkin_ws/src/tocabi_cc/data/data5.txt");
ofstream data6("/home/kwan/catkin_ws/src/tocabi_cc/data/data6.txt");

CustomController::CustomController(RobotData &rd) : rd_(rd) 
{
    nh_cc_.setCallbackQueue(&queue_cc_);

    ControlVal_.setZero();

    bool urdfmode = false;
    std::string urdf_path, desc_package_path;
    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);

    // if (urdfmode)
    // {
    //     urdf_path = desc_package_path + "/dyros_tocabi_ankleRollDamping.urdf";
    // }
    // else
    // {
    //     urdf_path = desc_package_path + "/dyros_tocabi.urdf";
    // }

    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_d_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_c_, true, false);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

double CustomController::getMpcFrequency() const
{
    return mpc_freq;
}

void CustomController::computeSlow()
{
    queue_cc_.callAvailable(ros::WallDuration());

    if (rd_.tc_.mode == 6)
    {   
        if (is_mode_6_init == true)
        {
            setGains();
            walkingParameterSetting();

            q_init_ = rd_.q_;

            walking_enable_ = true;

            cout << "COMPUTESLOW MODE 6 IS NOW INITIALIZED" << endl;
            cout << "TIME: "<< rd_.control_time_ << endl; 

            is_mode_6_init = false;
        }

        moveInitialPose();

        rd_.torque_desired = Kp_diag * (q_ref_ - rd_.q_) - Kd_diag * rd_.q_dot_;
    }
    else if (rd_.tc_.mode == 7)
    {
        if(walking_enable_ == true)
        {
            if (is_mode_7_init == true)
            {
                q_init_ = rd_.q_;
                q_leg_desired_ = rd_.q_.segment(0,12);

                contact_wrench_torque.setZero();
                lfoot_contact_wrench.setZero();
                rfoot_contact_wrench.setZero();

                cout << "COMPUTESLOW MODE 7 IS NOW INITIALIZED" << endl;
                is_mode_7_init = false;
            }

            updateInitialState();
            getRobotState();
            floatToSupportFootstep();

            if(current_step_num_ < total_step_num_)
            {
                walkingStateMachine();

                getZmpTrajectory();

                // getComTrajectory(); 
                getComTrajectory_mpc();

                getFootTrajectory(); 
                getPelvTrajectory();
                supportToFloatPattern();

                computeIkControl(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_leg_desired_);
                q_ref_.segment(0, 12) = q_leg_desired_;

                if (atb_grav_update_ == false)
                {
                    atb_grav_update_ = true;
                    atb_grav_update_ = false;
                }

                if (walking_tick < 1.0 * hz_)
                {
                        q_ref_.segment(0, 12) = DyrosMath::cubicVector<12>(walking_tick, 0, 1.0 * hz_, q_init_.segment(0,12), q_leg_desired_, Eigen::Vector12d::Zero(), Eigen::Vector12d::Zero());
                }

                data1 << ZMP_X_REF_ << "," << com_desired_(0) << "," << com_support_current_(0) << "," << cp_desired_(0) << "," << cp_measured_(0) << "," << rfoot_trajectory_support_.translation()(0) << "," << rfoot_support_current_.translation()(0) << "," << lfoot_trajectory_support_.translation()(0) << "," << lfoot_support_current_.translation()(0) << std::endl;
                data2 << ZMP_Y_REF_ << "," << com_desired_(1) << "," << com_support_current_(1) << "," << cp_desired_(1) << "," << cp_measured_(1) << "," << rfoot_trajectory_support_.translation()(1) << "," << rfoot_support_current_.translation()(1) << "," << lfoot_trajectory_support_.translation()(1) << "," << lfoot_support_current_.translation()(1) << std::endl;
                data3 << com_desired_(2) << "," << com_support_current_(2) << "," << rfoot_trajectory_support_.translation()(2) << "," << rfoot_support_current_.translation()(2) << "," << lfoot_trajectory_support_.translation()(2) << "," << lfoot_support_current_.translation()(2) << std::endl;

                updateNextStepTime();
                q_prev_ = rd_.q_;
            }
        }
        else
        {
            std::cout << "==================== WALKING FINISH ====================" << std::endl;
        }

        contactWrenchCalculator();

        Eigen::VectorQd torque_desired_temp;
        torque_desired_temp = Kp_diag * (q_ref_ - rd_.q_) - Kd_diag * rd_.q_dot_; 
        for(int i = 0; i < MODEL_DOF; i++) {torque_desired_temp(i) = DyrosMath::minmax_cut(torque_desired_temp(i), -200.0, 200.0);} // TODO : TORQUE MINMAX CUT SHO

        ///////////////////////////////FINAL TORQUE COMMAND/////////////////////////////
        rd_.torque_desired = Kp_diag * (q_ref_ - rd_.q_) - Kd_diag * rd_.q_dot_ + contact_wrench_torque; 
        ///////////////////////////////////////////////////////////////////////////////
    }
}

void CustomController::computeFast()
{
    if (rd_.tc_.mode == 6)
    {

    }
    else if (rd_.tc_.mode == 7)
    {

    }
}

void CustomController::computeThread3()
{
    static MPC mpc_(mpc_freq, mpc_N);
    
    if (rd_.tc_.mode == 7)
    {
        subDataSlowToThread3();

        int mpc_tick = walking_tick - zmp_start_time_;
        
        for (int i = 0; i < mpc_N; i++)
        {
            zx_ref(i) = ref_zmp_thread3(mpc_tick + int(hz_ / mpc_freq) * i, 0);
            zy_ref(i) = ref_zmp_thread3(mpc_tick + int(hz_ / mpc_freq) * i, 1);
        }

        mpc_.ComTrajectoryGenerator(zx_ref, zy_ref, x_mpc_thread3, y_mpc_thread3, com_height_);

        pubDataThread3ToSlow();
    }
}

void CustomController::computePlanner()
{

}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::setGains()
{
    Kp.setZero(); Kp_diag.setZero();
    Kd.setZero(); Kd_diag.setZero();

    ////////////////
    // JOINT GAIN //
    Kp(0) = 2000.0;
    Kd(0) = 20.0; // Left Hip yaw
    Kp(1) = 5000.0;
    Kd(1) = 55.0; // Left Hip roll
    Kp(2) = 4000.0;
    Kd(2) = 45.0; // Left Hip pitch
    Kp(3) = 3700.0;
    Kd(3) = 40.0; // Left Knee pitch
    Kp(4) = 4000.0; 
    Kd(4) = 65.0; // Left Ankle pitch
    Kp(5) = 4000.0; 
    Kd(5) = 65.0; // Left Ankle roll /5000 

    Kp(6) = 2000.0;
    Kd(6) = 20.0; // Right Hip yaw
    Kp(7) = 5000.0;
    Kd(7) = 55.0; // Right Hip roll 
    Kp(8) = 4000.0;
    Kd(8) = 45.0; // Right Hip pitch
    Kp(9) = 3700.0;
    Kd(9) = 40.0; // Right Knee pitch
    Kp(10) = 4000.0; 
    Kd(10) = 65.0; // Right Ankle pitch
    Kp(11) = 4000.0;
    Kd(11) = 65.0; // Right Ankle roll

    Kp(12) = 6000.0;
    Kd(12) = 200.0; // Waist yaw
    Kp(13) = 10000.0;
    Kd(13) = 100.0; // Waist pitch
    Kp(14) = 10000.0;
    Kd(14) = 100.0; // Waist roll

    Kp(15) = 400.0;
    Kd(15) = 10.0;
    Kp(16) = 800.0;
    Kd(16) = 10.0;
    Kp(17) = 400.0;
    Kd(17) = 10.0;
    Kp(18) = 400.0;
    Kd(18) = 10.0;
    Kp(19) = 250.0;
    Kd(19) = 2.5;
    Kp(20) = 250.0;
    Kd(20) = 2.0;
    Kp(21) = 50.0;
    Kd(21) = 2.0; // Left Wrist
    Kp(22) = 50.0;
    Kd(22) = 2.0; // Left Wrist

    Kp(23) = 50.0;
    Kd(23) = 2.0; // Neck
    Kp(24) = 50.0;
    Kd(24) = 2.0; // Neck

    Kp(25) = 400.0;
    Kd(25) = 10.0;
    Kp(26) = 800.0;
    Kd(26) = 10.0;
    Kp(27) = 400.0;
    Kd(27) = 10.0;
    Kp(28) = 400.0;
    Kd(28) = 10.0;
    Kp(29) = 250.0;
    Kd(29) = 2.5;
    Kp(30) = 250.0;
    Kd(30) = 2.0;
    Kp(31) = 50.0;
    Kd(31) = 2.0; // Right Wrist
    Kp(32) = 50.0;
    Kd(32) = 2.0; // Right Wrist

    Kp_diag = Kp.asDiagonal();
    Kd_diag = Kd.asDiagonal();

    /////////////////////
    // JOINT POS LIMIT //
    joint_limit_l_.setZero(MODEL_DOF);
    joint_limit_h_.setZero(MODEL_DOF);

    for (int i = 0; i < 12; i++)
    {
        joint_limit_l_(i) = -180 * DEG2RAD;
        joint_limit_h_(i) = 180 * DEG2RAD;
    }

    joint_limit_l_(3) = 20 * DEG2RAD;    // Left knee pitch
    joint_limit_h_(3) = 110 * DEG2RAD;   // Left knee pitch
    joint_limit_l_(9) = 20 * DEG2RAD;    // Right knee pitch
    joint_limit_h_(9) = 110 * DEG2RAD;   // Right knee pitch
    joint_limit_l_(12) =-30 * DEG2RAD;
    joint_limit_h_(12) = 30 * DEG2RAD;
    joint_limit_l_(13) =-15 * DEG2RAD;
    joint_limit_h_(13) = 15 * DEG2RAD;
    joint_limit_l_(14) =-15 * DEG2RAD;
    joint_limit_h_(14) = 15 * DEG2RAD;
    joint_limit_l_(15) =-30 * DEG2RAD;
    joint_limit_h_(15) = 20 * DEG2RAD;
    joint_limit_l_(16) =-50 * DEG2RAD;
    joint_limit_h_(16) = 50 * DEG2RAD;
    joint_limit_l_(17) = 45 * DEG2RAD;
    joint_limit_h_(17) = 65 * DEG2RAD;
    joint_limit_l_(18) =-90 * DEG2RAD;
    joint_limit_h_(18) =-65 * DEG2RAD;
    joint_limit_l_(19) =-150 * DEG2RAD;
    joint_limit_h_(19) =-10 * DEG2RAD;
    joint_limit_l_(20) =-180 * DEG2RAD;
    joint_limit_h_(20) = 180 * DEG2RAD;
    joint_limit_l_(21) =-70 * DEG2RAD;
    joint_limit_h_(21) = 70 * DEG2RAD;
    joint_limit_l_(22) =-60 * DEG2RAD;
    joint_limit_h_(22) = 60 * DEG2RAD;
    //HEAD
    joint_limit_l_(23) =-80 * DEG2RAD;
    joint_limit_h_(23) = 80 * DEG2RAD;
    joint_limit_l_(24) =-40 * DEG2RAD;
    joint_limit_h_(24) = 30 * DEG2RAD;
    //RIGHT ARM
    joint_limit_l_(25) =-20 * DEG2RAD;
    joint_limit_h_(25) = 30 * DEG2RAD;
    joint_limit_l_(26) =-50 * DEG2RAD;
    joint_limit_h_(26) = 50 * DEG2RAD;
    joint_limit_l_(27) =-65 * DEG2RAD;
    joint_limit_h_(27) =-45 * DEG2RAD;
    joint_limit_l_(28) = 65 * DEG2RAD;
    joint_limit_h_(28) = 90 * DEG2RAD;
    joint_limit_l_(29) = 10 * DEG2RAD;
    joint_limit_h_(29) = 150 * DEG2RAD;
    joint_limit_l_(30) =-180 * DEG2RAD;
    joint_limit_h_(30) = 180 * DEG2RAD;
    joint_limit_l_(31) =-70 * DEG2RAD;
    joint_limit_h_(31) = 70 * DEG2RAD;
    joint_limit_l_(32) =-60 * DEG2RAD;
    joint_limit_h_(32) = 60 * DEG2RAD;

    /////////////////////
    // JOINT VEL LIMIT //
    joint_vel_limit_l_.setZero(MODEL_DOF);
    joint_vel_limit_h_.setZero(MODEL_DOF);
    for (int i = 0; i < 12; i++)
    {
        joint_vel_limit_l_(i) = -2 * M_PI;
        joint_vel_limit_h_(i) =  2 * M_PI;
    }
    for (int i = 12; i < 33; i++)
    {
        joint_vel_limit_l_(i) = 0.0; // *2
        joint_vel_limit_h_(i) = 0.0; // *2
    }

    joint_vel_limit_l_(12) =-M_PI * 3.0;
    joint_vel_limit_h_(12) = M_PI * 3.0;
    joint_vel_limit_l_(13) =-M_PI / 6.0;
    joint_vel_limit_h_(13) = M_PI / 6.0;
    joint_vel_limit_l_(14) =-M_PI / 6.0;
    joint_vel_limit_h_(14) = M_PI / 6.0;
    joint_vel_limit_l_(15) =-M_PI * 1.5;
    joint_vel_limit_h_(15) = M_PI * 1.5;
    joint_vel_limit_l_(16) =-M_PI * 1.5;
    joint_vel_limit_h_(16) = M_PI * 1.5;
    joint_vel_limit_l_(17) =-M_PI * 1.5;
    joint_vel_limit_h_(17) = M_PI * 1.5;
    joint_vel_limit_l_(25) =-M_PI * 1.5;
    joint_vel_limit_h_(25) = M_PI * 1.5;
    joint_vel_limit_l_(26) =-M_PI * 1.5;
    joint_vel_limit_h_(26) = M_PI * 1.5;
    joint_vel_limit_l_(27) =-M_PI * 1.5;
    joint_vel_limit_h_(27) = M_PI * 1.5;
}

void CustomController::moveInitialPose()
{
    Eigen::VectorQd q_init_desired_; q_init_desired_.setZero();
    q_init_desired_ = q_init_;
    q_init_desired_(15) = + 15.0 * DEG2RAD; 
    q_init_desired_(16) = + 10.0 * DEG2RAD; 
    q_init_desired_(17) = + 80.0 * DEG2RAD; 
    q_init_desired_(18) = - 70.0 * DEG2RAD; 
    q_init_desired_(19) = - 45.0 * DEG2RAD; 
    q_init_desired_(21) =   0.0 * DEG2RAD; 

    q_init_desired_(25) = - 15.0 * DEG2RAD; 
    q_init_desired_(26) = - 10.0 * DEG2RAD;            
    q_init_desired_(27) = - 80.0 * DEG2RAD;  
    q_init_desired_(28) = + 70.0 * DEG2RAD; 
    q_init_desired_(29) = + 45.0 * DEG2RAD;       
    q_init_desired_(31) = - 0.0 * DEG2RAD; 

    q_ref_ = DyrosMath::cubicVector<MODEL_DOF>(initial_tick_, 0, 2.0 * hz_, q_init_, q_init_desired_, Eigen::VectorQd::Zero(), Eigen::VectorQd::Zero()); 

    initial_tick_++;
}

void CustomController::walkingParameterSetting()
{
    // BIPED WALKING
    ros::param::get("/tocabi_controller/target_x_", target_x_);
    ros::param::get("/tocabi_controller/target_y_", target_y_);
    ros::param::get("/tocabi_controller/target_z_", target_z_);
    ros::param::get("/tocabi_controller/step_length_x_", step_length_x_);
    ros::param::get("/tocabi_controller/step_length_y_", step_length_y_);
    ros::param::get("/tocabi_controller/com_height_", com_height_);
    ros::param::get("/tocabi_controller/is_right_foot_swing_", is_right_foot_swing_);

    ros::param::get("/tocabi_controller/t_rest_init_", t_rest_init_); 
    ros::param::get("/tocabi_controller/t_rest_last_", t_rest_last_);
    ros::param::get("/tocabi_controller/t_double1_", t_double1_);
    ros::param::get("/tocabi_controller/t_double2_", t_double2_);
    ros::param::get("/tocabi_controller/t_total_", t_total_);
    ros::param::get("/tocabi_controller/t_temp_", t_temp_);

    ros::param::get("/tocabi_controller/foot_height_", foot_height_);

    // ZMP CTRL
    ros::param::get("/tocabi_controller/kp_cp", kp_cp);

    // ZMP OFFSET
    ros::param::get("/tocabi_controller/zmp_offset", zmp_offset);

    t_rest_init_ = t_rest_init_ * hz_;
    t_rest_last_ = t_rest_last_ * hz_;
    t_double1_ = t_double1_ * hz_;
    t_double2_ = t_double2_ * hz_;
    t_total_ = t_total_ * hz_;
    t_temp_ = t_temp_ * hz_;

    t_dsp1_ = t_rest_init_ + t_double1_;
    t_dsp2_ = t_rest_last_ + t_double2_;
    t_ssp_ = t_total_ - (t_dsp1_ + t_dsp2_);

    t_last_ = t_total_ + t_temp_;
    t_start_ = t_temp_ + 1;
    t_start_real_ = t_start_ + t_rest_init_;

    current_step_num_ = 0;
}

void CustomController::updateInitialState()
{
    if (walking_tick == 0)
    {
        calculateFootStepTotal();

        pelv_rpy_current_.setZero();
        pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Pelvis].xpos);

        pelv_float_init_.translation()(0) = 0;
        pelv_float_init_.translation()(1) = 0;

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        lfoot_float_init_.translation()(0) = 0;
        lfoot_float_init_.translation()(1) = 0.1225;

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        rfoot_float_init_.translation()(0) = 0;
        rfoot_float_init_.translation()(1) = -0.1225;

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        com_float_init_(0) = 0;
        com_float_init_(1) = 0;

        if (aa == 0)
        {
            lfoot_float_init_.translation()(1) = 0.1025;
            rfoot_float_init_.translation()(1) = -0.1025;
            aa = 1;
        }

        Eigen::Isometry3d ref_frame;

        if (foot_step_(0, 6) == 0) //right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(0, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), rfoot_float_init_);
        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);

        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

        supportfoot_float_init_.setZero();
        swingfoot_float_init_.setZero();

        if (foot_step_(0, 6) == 1) //left suppport foot
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }
        else
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }

        total_step_num_ = foot_step_.col(1).size();
    }
    else if (current_step_num_ != 0 && is_support_foot_change == true) // step change
    {
        pelv_rpy_current_.setZero();
        pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));

        rfoot_rpy_current_.setZero();
        lfoot_rpy_current_.setZero();
        rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
        lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

        rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
        lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
        rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
        lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Pelvis].xpos);

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        Eigen::Isometry3d ref_frame;

        if (foot_step_(current_step_num_, 6) == 0) //right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        Eigen::Isometry3d ref_frame_yaw_only;
        ref_frame_yaw_only.translation() = ref_frame.translation();
        Eigen::Vector3d ref_frame_rpy;
        ref_frame_rpy = DyrosMath::rot2Euler(ref_frame.linear());
        ref_frame_yaw_only.linear() = DyrosMath::rotateWithZ(ref_frame_rpy(2));

        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame_yaw_only) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), com_float_init_);
        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), rfoot_float_init_);
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

        is_support_foot_change = false;
    }
}

void CustomController::getRobotState()
{
    pelv_rpy_current_.setZero();
    pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

    R_angle = pelv_rpy_current_(0);
    P_angle = pelv_rpy_current_(1);
    pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));

    rfoot_rpy_current_.setZero();
    lfoot_rpy_current_.setZero();
    rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
    lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

    rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
    lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
    rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
    lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

    pelv_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].rotm;

    pelv_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Pelvis].xpos);

    lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].rotm;
    // lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm;
    lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

    rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].rotm;
    // rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm;
    rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

    com_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치
    com_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].v);

    cp_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].xpos + rd_.link_[COM_id].v / wn);

    if (walking_tick == 0)
    {
        com_float_current_dot_LPF = com_float_current_dot;
        com_float_current_dot_prev = com_float_current_dot;
    }

    com_float_current_dot_prev = com_float_current_dot;
    com_float_current_dot_LPF = 1 / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot_LPF + (2 * M_PI * 3.0 * del_t) / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot;

    if (walking_tick == 0)
    {
        com_float_current_LPF = com_float_current_;
    }

    com_float_current_LPF = 1 / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_LPF + (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_;

    double support_foot_flag = foot_step_(current_step_num_, 6);
    if (support_foot_flag == 0)
    {
        supportfoot_float_current_ = rfoot_float_current_;
    }
    else if (support_foot_flag == 1)
    {
        supportfoot_float_current_ = lfoot_float_current_;
    }

    ///////////dg edit
    Eigen::Isometry3d supportfoot_float_current_yaw_only;
    supportfoot_float_current_yaw_only.translation() = supportfoot_float_current_.translation();
    Eigen::Vector3d support_foot_current_rpy;
    support_foot_current_rpy = DyrosMath::rot2Euler(supportfoot_float_current_.linear());
    supportfoot_float_current_yaw_only.linear() = DyrosMath::rotateWithZ(support_foot_current_rpy(2));

    pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * pelv_float_current_;
    lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * lfoot_float_current_;
    rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * rfoot_float_current_;

    com_support_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_);
    com_support_current_dot_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot);
    com_support_current_dot_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot_LPF);

    // cp_measured_(0) = com_support_current_(0) + com_float_current_dot_LPF(0) / wn;
    // cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1) / wn;
    cp_measured_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), cp_float_current_).head(2);
 
    // l_ft : generated force by robot
    l_ft_ = rd_.LF_FT;
    r_ft_ = rd_.RF_FT;

    if (walking_tick == 0)
    {
        l_ft_LPF = l_ft_;
        r_ft_LPF = r_ft_;
    }

    l_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_;
    r_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_;
 
    Eigen::Vector2d left_zmp, right_zmp;

    left_zmp(0) = l_ft_LPF(4) / l_ft_LPF(2) + lfoot_support_current_.translation()(0);
    left_zmp(1) = l_ft_LPF(3) / l_ft_LPF(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = r_ft_LPF(4) / r_ft_LPF(2) + rfoot_support_current_.translation()(0);
    right_zmp(1) = r_ft_LPF(3) / r_ft_LPF(2) + rfoot_support_current_.translation()(1);

    zmp_measured_mj_(0) = (left_zmp(0) * l_ft_LPF(2) + right_zmp(0) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP X
    zmp_measured_mj_(1) = (left_zmp(1) * l_ft_LPF(2) + right_zmp(1) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP Y
    wn = sqrt(GRAVITY / com_height_);

    if (walking_tick == 0)
    {
        zmp_measured_LPF_.setZero();
    } 
    zmp_measured_LPF_ = (2 * M_PI * 2.0 * del_t) / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_mj_ + 1 / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_LPF_;
}

void CustomController::calculateFootStepTotal()
{
    double initial_rot = 0.0;
    double final_rot = 0.0;
    double initial_drot = 0.0;
    double final_drot = 0.0;

    initial_rot = atan2(target_y_, target_x_);

    if (initial_rot > 0.0)
        initial_drot = 20 * DEG2RAD;
    else
        initial_drot = -20 * DEG2RAD;

    unsigned int initial_total_step_number = initial_rot / initial_drot;
    double initial_residual_angle = initial_rot - initial_total_step_number * initial_drot;

    final_rot = target_theta_ - initial_rot;
    if (final_rot > 0.0)
        final_drot = 20 * DEG2RAD;
    else
        final_drot = -20 * DEG2RAD;

    unsigned int final_total_step_number = final_rot / final_drot;
    double final_residual_angle = final_rot - final_total_step_number * final_drot;
    double length_to_target = sqrt(target_x_ * target_x_ + target_y_ * target_y_);
    double dlength = step_length_x_;
    unsigned int middle_total_step_number = length_to_target / dlength;
    double middle_residual_length = length_to_target - middle_total_step_number * dlength;

    double step_width_init;
    double step_width;

    step_width_init = 0.01;
    step_width = 0.02;

    if (length_to_target == 0.0)
    {
        middle_total_step_number = 20; //total foot step number
        dlength = 0;
    }

    unsigned int number_of_foot_step;

    int del_size;

    del_size = 1;
    number_of_foot_step = 2 + initial_total_step_number * del_size + middle_total_step_number * del_size + final_total_step_number * del_size;
    
    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
    {
        if (initial_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(initial_residual_angle) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
    {
        if (middle_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(middle_residual_length) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        if (abs(final_residual_angle) >= 0.0001)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
            number_of_foot_step = number_of_foot_step + del_size;
    }

    foot_step_.resize(number_of_foot_step, 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(number_of_foot_step, 7);
    foot_step_support_frame_.setZero();
    
    int index = 0;
    int temp, temp2, temp3, is_right;

    if (is_right_foot_swing_ == true)
        is_right = 1;
    else
        is_right = -1;

    temp = -is_right;
    temp2 = -is_right;
    temp3 = -is_right;

    int temp0;
    temp0 = -is_right;

    double initial_dir = 0.0;

    if (aa == 0)
    {
        for (int i = 0; i < 2; i++)
        {
            temp0 *= -1;

            if (i == 0)
            {
                foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (0.1025 + step_width_init * (i + 1));
                foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (0.1025 + step_width_init * (i + 1));
            }
            else if (i == 1)
            {
                foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (0.1025 + step_width_init * (i + 1));
                foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (0.1025 + step_width_init * (i + 1));
            }

            foot_step_(index, 5) = initial_dir;
            foot_step_(index, 6) = 0.5 + 0.5 * temp0;
            index++;
        }
    }
    else if (aa == 1)
    {
        for (int i = 0; i < 2; i++)
        {
            temp0 *= -1;

            foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_dir;
            foot_step_(index, 6) = 0.5 + 0.5 * temp0;
            index++;
        }
    }

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001) // 첫번째 회전
    {
        for (int i = 0; i < initial_total_step_number; i++)
        {
            temp *= -1;
            foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((i + 1) * initial_drot);
            foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((i + 1) * initial_drot);
            foot_step_(index, 5) = (i + 1) * initial_drot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }

        if (temp == is_right)
        {
            if (abs(initial_residual_angle) >= 0.0001)
            {
                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
            else
            {
                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
        }
        else if (temp == -is_right)
        {
            temp *= -1;

            foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;

            temp *= -1;

            foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001) // 직진, 제자리 보행
    {

        for (int i = 0; i < middle_total_step_number; i++)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (i + 1)) + temp2 * sin(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (i + 1)) - temp2 * cos(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }

        if (temp2 == is_right)
        {
            if (abs(middle_residual_length) >= 0.0001)
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;

                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
            else
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
        }
        else if (temp2 == -is_right)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;

            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }
    }

    double final_position_x = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);
    double final_position_y = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        for (int i = 0; i < final_total_step_number; i++)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin((i + 1) * final_drot + initial_rot);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos((i + 1) * final_drot + initial_rot);
            foot_step_(index, 5) = (i + 1) * final_drot + initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }

        if (abs(final_residual_angle) >= 0.0001)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;

            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
        else
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
    }
    
    //
    bool Forward = false, Side = false;

    if(Forward)
    {
        number_of_foot_step = 11;
        foot_step_.resize(number_of_foot_step, 7);
        foot_step_.setZero();
        foot_step_support_frame_.resize(number_of_foot_step, 7);
        foot_step_support_frame_.setZero();

        // Forward
        foot_step_(0, 0) = 0.0; foot_step_(0, 1) = -0.1125; foot_step_(0, 6) = 1.0; 
        foot_step_(1, 0) = 0.0; foot_step_(1, 1) =  0.1225; foot_step_(1, 6) = 0.0;
        foot_step_(2, 0) = 0.05; foot_step_(2, 1) = -0.1225; foot_step_(2, 6) = 1.0; 
        foot_step_(3, 0) = 0.15; foot_step_(3, 1) =  0.1225; foot_step_(3, 6) = 0.0;
        foot_step_(4, 0) = 0.30; foot_step_(4, 1) = -0.1225; foot_step_(4, 6) = 1.0; 
        foot_step_(5, 0) = 0.15; foot_step_(5, 1) =  0.1225; foot_step_(5, 6) = 0.0;
        foot_step_(6, 0) = 0.30; foot_step_(6, 1) = -0.1225; foot_step_(6, 6) = 1.0; 
        foot_step_(7, 0) = 0.15; foot_step_(7, 1) =  0.1225; foot_step_(7, 6) = 0.0;
        foot_step_(8, 0) = 0.05; foot_step_(8, 1) = -0.1225; foot_step_(8, 6) = 1.0;
        foot_step_(9, 0) = 0.0; foot_step_(9, 1) =  0.1225; foot_step_(9, 6) = 0.0; 
        foot_step_(10, 0) = 0.0; foot_step_(10, 1) = -0.1225; foot_step_(10, 6) = 1.0; 
    }
    if(Side)
    {
        number_of_foot_step = 11;
        foot_step_.resize(number_of_foot_step, 7);
        foot_step_.setZero();
        foot_step_support_frame_.resize(number_of_foot_step, 7);
        foot_step_support_frame_.setZero();

        foot_step_(0, 0) = 0.0; foot_step_(0, 1) = -0.1125; foot_step_(0, 6) = 1.0; 
        foot_step_(1, 0) = 0.0; foot_step_(1, 1) =  0.1225; foot_step_(1, 6) = 0.0;
        foot_step_(2, 0) = 0.0; foot_step_(2, 1) = -0.1225; foot_step_(2, 6) = 1.0;
        foot_step_(3, 0) = 0.0; foot_step_(3, 1) =  0.1225 + 0.05*2; foot_step_(3, 6) = 0.0;
        foot_step_(4, 0) = 0.0; foot_step_(4, 1) = -0.1225 - 0.05*2; foot_step_(4, 6) = 1.0;
        foot_step_(5, 0) = 0.0; foot_step_(5, 1) =  0.1225 + 0.05*2*0; foot_step_(5, 6) = 0.0;
        foot_step_(6, 0) = 0.0; foot_step_(6, 1) = -0.1225 - 0.05*2*0; foot_step_(6, 6) = 1.0; 
        foot_step_(7, 0) = 0.0; foot_step_(7, 1) =  0.1225 + 0.05*2; foot_step_(7, 6) = 0.0; 
        foot_step_(8, 0) = 0.0; foot_step_(8, 1) = -0.1225 - 0.05*2; foot_step_(8, 6) = 1.0;
        foot_step_(9, 0) = 0.0; foot_step_(9, 1) =  0.1225; foot_step_(9, 6) = 0.0;
        foot_step_(10, 0) = 0.0; foot_step_(10, 1) = -0.1225; foot_step_(10, 6) = 1.0;
    }
       
    
}

void CustomController::floatToSupportFootstep()
{
    Eigen::Isometry3d reference;

    if (current_step_num_ == 0)
    {
        if (foot_step_(0, 6) == 0)
        {
            reference.translation() = rfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
        else
        {
            reference.translation() = lfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
    }
    else
    {
        reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_ - 1, 5));
        for (int i = 0; i < 3; i++)
        {
            reference.translation()(i) = foot_step_(current_step_num_ - 1, i);
        }
    }

    Eigen::Vector3d temp_local_position;
    Eigen::Vector3d temp_global_position;

    for (int i = 0; i < total_step_num_; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            temp_global_position(j) = foot_step_(i, j);
        }

        temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

        for (int j = 0; j < 3; j++)
        {
            foot_step_support_frame_(i, j) = temp_local_position(j);
        }

        foot_step_support_frame_(i, 3) = foot_step_(i, 3);
        foot_step_support_frame_(i, 4) = foot_step_(i, 4);
        if (current_step_num_ == 0)
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - supportfoot_float_init_(5);
        }
        else
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - foot_step_(current_step_num_ - 1, 5);
        }
    }

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = swingfoot_float_init_(j); // swingfoot_float_init_은 Pelvis에서 본 Swing 발의 Position, orientation.

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        swingfoot_support_init_(j) = temp_local_position(j);

    swingfoot_support_init_(3) = swingfoot_float_init_(3);
    swingfoot_support_init_(4) = swingfoot_float_init_(4);

    if (current_step_num_ == 0)
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - supportfoot_float_init_(5);
    else
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = supportfoot_float_init_(j);

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        supportfoot_support_init_(j) = temp_local_position(j);

    supportfoot_support_init_(3) = supportfoot_float_init_(3);
    supportfoot_support_init_(4) = supportfoot_float_init_(4);

    if (current_step_num_ == 0)
        supportfoot_support_init_(5) = 0;
    else
        supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);
}

void CustomController::getZmpTrajectory()
{
    unsigned int planning_step_number = 5;
    unsigned int norm_size = 0;

    // LOCAL TIME
    if (current_step_num_ == 0){ zmp_start_time_ = 0; }
    else { zmp_start_time_ = t_start_; }

    if (current_step_num_ >= total_step_num_ - planning_step_number)
    {
        norm_size = t_total_ * (total_step_num_ - current_step_num_) + 4.0 * hz_;        
    }
    else
    {
        norm_size = t_total_ * planning_step_number + 1.0 * hz_;
    }         
    if (current_step_num_ == 0)
    {
        norm_size = norm_size + t_temp_ + 1;
    }        
    
    addZmpOffset(); 
    zmpGenerator(norm_size, planning_step_number);

    ZMP_X_REF_ = ref_zmp_(walking_tick - zmp_start_time_, 0);
    ZMP_Y_REF_ = ref_zmp_(walking_tick - zmp_start_time_, 1); 
}

void CustomController::addZmpOffset()
{
    double lfoot_zmp_offset_, rfoot_zmp_offset_;

    lfoot_zmp_offset_ = -zmp_offset; // simul 1.1 s
    rfoot_zmp_offset_ =  zmp_offset;

    foot_step_support_frame_offset_ = foot_step_support_frame_;

    supportfoot_support_init_offset_ = supportfoot_support_init_;

    if (foot_step_(0, 6) == 0) //right support foot
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
    }
    else
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_; 
    }       

    for (int i = 0; i < total_step_num_; i++)
    {
        if (foot_step_(i, 6) == 0) // left support foot 
        {
            foot_step_support_frame_offset_(i, 1) += lfoot_zmp_offset_;
        }
        else // right support foot
        {
            foot_step_support_frame_offset_(i, 1) += rfoot_zmp_offset_;
        }
    }
}

void CustomController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
    /*
    Goal
    ----------
    -> To position the ZMP at the center of the foot sole according to the footstep planning to prevent the robot from falling during walking.

    Parameters
    ----------
    -> norm_size : The size of the previewed vector for the ZMP reference.

    -> planning_step_num : The number of footsteps to be predicted.

    Returns
    -------
    -> ref_zmp_ : The ZMP reference vector calculated based on the foot sole plan (size: 2 x norm_size).
    */

    ref_zmp_.setZero(norm_size, 2);
    ref_zmp_thread3.setZero(norm_size, 2);

    Eigen::VectorXd temp_px;
    Eigen::VectorXd temp_py;

    unsigned int index = 0;

    if (current_step_num_ == 0)  
    {
        for (int i = 0; i <= t_temp_; i++) 
        {
            if (i < 1.0 * hz_)
            {
                ref_zmp_(i, 0) = com_support_init_(0);
                ref_zmp_(i, 1) = com_support_init_(1);
            }
            else if (i < 2.0 * hz_)
            {
                ref_zmp_(i, 0) = DyrosMath::cubic(i, 1.0 * hz_, 2.0 * hz_, com_support_init_(0), 0.0, 0.0, 0.0);
                ref_zmp_(i, 1) = com_support_init_(1);
            }
            else
            {
                ref_zmp_(i, 0) = 0.0;
                ref_zmp_(i, 1) = com_support_init_(1);
            }
            index++;
        }
    }

    if(current_step_num_ >= total_step_num_ - planning_step_num)
    {   
        for(unsigned int i = current_step_num_; i < total_step_num_; i++)
        {   
            onestepZmp(i, temp_px, temp_py);
           
            ref_zmp_.block(index, 0, t_total_, 1) = temp_px; 
            ref_zmp_.block(index, 1, t_total_, 1) = temp_py;
           
            index = index + t_total_;
        }

        ref_zmp_.block(index, 0, 3 * hz_, 1).setConstant(ref_zmp_(index - 1, 0));
        ref_zmp_.block(index, 1, 3 * hz_, 1).setConstant(ref_zmp_(index - 1, 1));

        index = index + 3.0 * hz_; // Norm size must be larger than this addtional zmp size.
    }
    else // reference ZMP during walking
    {       
        for (unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)
        {   
            onestepZmp(i, temp_px, temp_py); // save 1-step zmp into temp px, py
             
            ref_zmp_.block(index, 0, t_total_, 1) = temp_px; 
            ref_zmp_.block(index, 1, t_total_, 1) = temp_py;
            
            index = index + t_total_;                                                          
        }
    }
}

void CustomController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py)
{
    temp_px.setZero(t_total_);  
    temp_py.setZero(t_total_);
    
    double v0_x_dsp1 = 0.0; double v0_y_dsp1 = 0.0;
    double vT_x_dsp1 = 0.0; double vT_y_dsp1 = 0.0;
    double v0_x_ssp  = 0.0; double v0_y_ssp = 0.0;
    double vT_x_ssp  = 0.0; double vT_y_ssp = 0.0;
    double v0_x_dsp2 = 0.0; double v0_y_dsp2 = 0.0;
    double vT_x_dsp2 = 0.0; double vT_y_dsp2 = 0.0;
    
    if (current_step_number == 0)
    {
        v0_x_dsp1 = 0.0;
        vT_x_dsp1 = 0.0;
        v0_y_dsp1 = com_support_init_(1);
        vT_y_dsp1 = supportfoot_support_init_offset_(1);

        v0_x_ssp = 0.0;
        vT_x_ssp = 0.0;
        v0_y_ssp = supportfoot_support_init_offset_(1);
        vT_y_ssp = supportfoot_support_init_offset_(1);

        v0_x_dsp2 = 0.0;
        vT_x_dsp2 = (foot_step_support_frame_offset_(current_step_number - 0, 0) + supportfoot_support_init_offset_(0)) / 2.0;
        v0_y_dsp2 = supportfoot_support_init_offset_(1);
        vT_y_dsp2 = (foot_step_support_frame_offset_(current_step_number - 0, 1) + supportfoot_support_init_offset_(1)) / 2.0;
    }
    else if (current_step_number == 1)
    { 
        v0_x_dsp1 = (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_offset_(0)) / 2.0;
        vT_x_dsp1 =  foot_step_support_frame_offset_(current_step_number - 1, 0);
        v0_y_dsp1 = (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_offset_(1)) / 2.0;
        vT_y_dsp1 =  foot_step_support_frame_offset_(current_step_number - 1, 1);

        v0_x_ssp = foot_step_support_frame_offset_(current_step_number - 1, 0);
        vT_x_ssp = foot_step_support_frame_offset_(current_step_number - 1, 0);
        v0_y_ssp = foot_step_support_frame_offset_(current_step_number - 1, 1);
        vT_y_ssp = foot_step_support_frame_offset_(current_step_number - 1, 1);

        v0_x_dsp2 =  foot_step_support_frame_offset_(current_step_number - 1, 0);
        vT_x_dsp2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2.0;
        v0_y_dsp2 =  foot_step_support_frame_offset_(current_step_number - 1, 1);
        vT_y_dsp2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2.0;
    }
    else
    {   
        v0_x_dsp1 = (foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2.0;
        vT_x_dsp1 =  foot_step_support_frame_offset_(current_step_number - 1, 0);
        v0_y_dsp1 = (foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2.0;
        vT_y_dsp1 =  foot_step_support_frame_offset_(current_step_number - 1, 1);

        v0_x_ssp = foot_step_support_frame_offset_(current_step_number - 1, 0);
        vT_x_ssp = foot_step_support_frame_offset_(current_step_number - 1, 0);
        v0_y_ssp = foot_step_support_frame_offset_(current_step_number - 1, 1);
        vT_y_ssp = foot_step_support_frame_offset_(current_step_number - 1, 1);

        v0_x_dsp2 =  foot_step_support_frame_offset_(current_step_number - 1, 0);
        vT_x_dsp2 = (foot_step_support_frame_offset_(current_step_number - 1, 0) + foot_step_support_frame_offset_(current_step_number - 0, 0)) / 2.0;
        v0_y_dsp2 =  foot_step_support_frame_offset_(current_step_number - 1, 1);
        vT_y_dsp2 = (foot_step_support_frame_offset_(current_step_number - 1, 1) + foot_step_support_frame_offset_(current_step_number - 0, 1)) / 2.0;
    }

    double lin_interpol = 0.0;
    for (int i = 0; i < t_total_; i++)
    {
        if (i < t_dsp1_) 
        { 
            lin_interpol = i / (t_dsp1_);
            temp_px(i) = (1.0 - lin_interpol) * v0_x_dsp1 + lin_interpol * vT_x_dsp1;
            temp_py(i) = (1.0 - lin_interpol) * v0_y_dsp1 + lin_interpol * vT_y_dsp1;
            
            // temp_px(i) = DyrosMath::cubic(i, 0.0, t_dsp1_, v0_x_dsp1, vT_x_dsp1, 0.0, 0.0);
            // temp_py(i) = DyrosMath::cubic(i, 0.0, t_dsp1_, v0_y_dsp1, vT_y_dsp1, 0.0, 0.0);
        }
        else if (i >= t_dsp1_ && i < t_dsp1_ + t_ssp_)
        {
            lin_interpol = (i - t_dsp1_) / t_ssp_;
            temp_px(i) = (1.0 - lin_interpol) * v0_x_ssp + lin_interpol * vT_x_ssp;
            temp_py(i) = (1.0 - lin_interpol) * v0_y_ssp + lin_interpol * vT_y_ssp;

            // temp_px(i) = DyrosMath::cubic(i, t_dsp1_, t_dsp1_ + t_ssp_, v0_x_ssp, vT_x_ssp, 0.0, 0.0);
            // temp_py(i) = DyrosMath::cubic(i, t_dsp1_, t_dsp1_ + t_ssp_, v0_y_ssp, vT_y_ssp, 0.0, 0.0);
        }
        else
        {
            lin_interpol = (i - t_dsp1_ - t_ssp_) / t_dsp2_;
            temp_px(i) = (1.0 - lin_interpol) * v0_x_dsp2 + lin_interpol * vT_x_dsp2;
            temp_py(i) = (1.0 - lin_interpol) * v0_y_dsp2 + lin_interpol * vT_y_dsp2;

            // temp_px(i) = DyrosMath::cubic(i, t_dsp1_ + t_ssp_, t_total_, v0_x_dsp2, vT_x_dsp2, 0.0, 0.0);
            // temp_py(i) = DyrosMath::cubic(i, t_dsp1_ + t_ssp_, t_total_, v0_y_dsp2, vT_y_dsp2, 0.0, 0.0);
        }
    }

}

void CustomController::getComTrajectory()
{
    double dt_preview_ = 1.0 / hz_; // : sampling time of preview [s]
    double NL_preview  = 3200;      // : number of preview horizons

    if (is_preview_ctrl_init == true)
    {
        Gi_preview_.setZero();
        Gd_preview_.setZero();
        Gx_preview_.setZero();

        preview_Parameter(dt_preview_, NL_preview, Gi_preview_, Gd_preview_, Gx_preview_, A_preview_, B_preview_, C_preview_);
        
        x_preview_.setZero(); y_preview_.setZero(); 
        x_preview_(0) = com_support_init_(0);
        y_preview_(0) = com_support_init_(1);

        UX_preview_ = 0;
        UY_preview_ = 0;

        std::cout << "PREVIEW PARAMETERS ARE SUCCESSFULLY INITIALIZED" << std::endl;
    }

    previewcontroller(dt_preview_, NL_preview, walking_tick - zmp_start_time_, 
                      x_preview_, y_preview_, UX_preview_, UY_preview_,
                      Gi_preview_, Gd_preview_, Gx_preview_, 
                      A_preview_, B_preview_, C_preview_);

    com_desired_(0) = x_preview_(0);
    com_desired_(1) = y_preview_(0);
    com_desired_(2) = com_height_;
    
    if (walking_tick == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1)
    {
        Eigen::Vector3d com_pos_prev;
        Eigen::Vector3d com_pos;
        Eigen::Vector3d com_vel_prev;
        Eigen::Vector3d com_vel;
        Eigen::Vector3d com_acc_prev;
        Eigen::Vector3d com_acc;
        Eigen::Matrix3d temp_rot;
        Eigen::Vector3d temp_pos;

        temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_, 5));
        for (int i = 0; i < 3; i++)
            temp_pos(i) = foot_step_support_frame_(current_step_num_, i);
        
        com_pos_prev(0) = x_preview_(0);
        com_pos_prev(1) = y_preview_(0);
        com_pos = temp_rot * (com_pos_prev - temp_pos);

        com_vel_prev(0) = x_preview_(1);
        com_vel_prev(1) = y_preview_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = x_preview_(2);
        com_acc_prev(1) = y_preview_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        x_preview_(0) = com_pos(0);
        y_preview_(0) = com_pos(1);
        x_preview_(1) = com_vel(0);
        y_preview_(1) = com_vel(1);
        x_preview_(2) = com_acc(0);
        y_preview_(2) = com_acc(1);
    }
}

void CustomController::preview_Parameter(double dt, int NL, Eigen::MatrixXd &Gi, Eigen::VectorXd &Gd, Eigen::MatrixXd &Gx, Eigen::MatrixXd &A, Eigen::VectorXd &B, Eigen::MatrixXd &C)
{
    A.resize(3, 3);
    A(0, 0) = 1.0;
    A(0, 1) = dt;
    A(0, 2) = dt * dt * 0.5;
    A(1, 0) = 0;
    A(1, 1) = 1.0;
    A(1, 2) = dt;
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

    B.resize(3);
    B(0) = dt * dt * dt / 6;
    B(1) = dt * dt / 2;
    B(2) = dt;

    C.resize(1, 3);
    C(0, 0) = 1;
    C(0, 1) = 0;
    C(0, 2) = -com_height_ / GRAVITY;

    Eigen::MatrixXd A_bar;
    Eigen::VectorXd B_bar;

    B_bar.setZero(4);
    B_bar.segment(0, 1) = C * B;
    B_bar.segment(1, 3) = B;

    Eigen::Matrix1x4d B_bar_tran;
    B_bar_tran = B_bar.transpose();

    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.setZero(4, 4);
    I_bar.setZero(4, 1);
    F_bar.setZero(4, 3);

    F_bar.block<1, 3>(0, 0) = C * A;
    F_bar.block<3, 3>(1, 0) = A;

    I_bar.setZero();
    I_bar(0, 0) = 1.0;

    A_bar.block<4, 1>(0, 0) = I_bar;
    A_bar.block<4, 3>(0, 1) = F_bar;

    Eigen::MatrixXd Qe;
    Qe.setZero(1, 1);
    Qe(0, 0) = 1.0;

    Eigen::MatrixXd R;
    R.setZero(1, 1);
    R(0, 0) = 0.000001;

    Eigen::MatrixXd Qx;
    Qx.setZero(3, 3);

    Eigen::MatrixXd Q_bar;
    Q_bar.setZero(3, 3);
    Q_bar(0, 0) = Qe(0, 0);

    Eigen::Matrix4d K; K.setZero();

    K(0, 0) = 1083.572780788710;
    K(0, 1) = 586523.188429418020;
    K(0, 2) = 157943.283121116518;
    K(0, 3) = 41.206077691894;
    K(1, 0) = 586523.188429418020;
    K(1, 1) = 319653984.254277825356;
    K(1, 2) = 86082274.531361579895;
    K(1, 3) = 23397.754069026785;
    K(2, 0) = 157943.283121116518;
    K(2, 1) = 86082274.531361579895;
    K(2, 2) = 23181823.112113621086;
    K(2, 3) = 6304.466397614751;
    K(3, 0) = 41.206077691894;
    K(3, 1) = 23397.754069026785;
    K(3, 2) = 6304.466397614751;
    K(3, 3) = 2.659250532188;

    Eigen::MatrixXd Temp_mat;
    Eigen::MatrixXd Temp_mat_inv;
    Eigen::MatrixXd Ac_bar;
    Temp_mat.setZero(1, 1);
    Temp_mat_inv.setZero(1, 1);
    Ac_bar.setZero(4, 4);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse();

    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;

    Eigen::MatrixXd Ac_bar_tran(4, 4);
    Ac_bar_tran = Ac_bar.transpose();

    Gi.setZero(1, 1);
    Gx.setZero(1, 3);
    Gi(0, 0) = 872.3477; //Temp_mat_inv * B_bar_tran * K * I_bar ;
    //Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;
    Gx(0, 0) = 945252.1760702;
    Gx(0, 1) = 256298.6905049;
    Gx(0, 2) = 542.0544196;
    Eigen::MatrixXd X_bar;
    Eigen::Vector4d X_bar_col;
    X_bar.setZero(4, NL);
    X_bar_col.setZero();
    X_bar_col = -Ac_bar_tran * K * I_bar;

    for (int i = 0; i < NL; i++)
    {
        X_bar.block<4, 1>(0, i) = X_bar_col;
        X_bar_col = Ac_bar_tran * X_bar_col;
    }

    Gd.setZero(NL);
    Eigen::VectorXd Gd_col(1);
    Gd_col(0) = -Gi(0, 0);

    for (int i = 0; i < NL; i++)
    {
        Gd.segment(i, 1) = Gd_col;
        Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i);
    }
}

void CustomController::previewcontroller(double dt, int NL, int tick, 
                                         Eigen::Vector3d &x_k, Eigen::Vector3d &y_k, double &UX, double &UY,
                                         const Eigen::MatrixXd &Gi, const Eigen::VectorXd &Gd, const Eigen::MatrixXd &Gx, 
                                         const Eigen::MatrixXd &A,  const Eigen::VectorXd &B,  const Eigen::MatrixXd &C)
{
    Eigen::Vector3d x_k_prev_; x_k_prev_.setZero();
    Eigen::Vector3d y_k_prev_; y_k_prev_.setZero();

    if(is_preview_ctrl_init == true)
    {
        x_k_prev_ = x_k;
        y_k_prev_ = y_k; // To prevent the ZMP overshooting.

        is_preview_ctrl_init = false;
    }
    else
    {
        x_k_prev_(0) = x_k(0) - x_k(1) * dt;
        x_k_prev_(1) = x_k(1) - x_k(2) * dt;
        x_k_prev_(2) = x_k(2) -     UX * dt;

        y_k_prev_(0) = y_k(0) - y_k(1) * dt;
        y_k_prev_(1) = y_k(1) - y_k(2) * dt;
        y_k_prev_(2) = y_k(2) -     UY * dt;
    }

    Eigen::VectorXd px, py;
    px.setZero(1); px = C * x_k;
    py.setZero(1); py = C * y_k;

    double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;
    for (int i = 0; i < NL; i++)
    {
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i) * (ref_zmp_(tick + 1 + i,0) - ref_zmp_(tick + i,0));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i) * (ref_zmp_(tick + 1 + i,1) - ref_zmp_(tick + i,1));
    }

    Eigen::MatrixXd del_ux; del_ux.setZero(1, 1);
    Eigen::MatrixXd del_uy; del_uy.setZero(1, 1);

    Eigen::VectorXd GX_X; GX_X.setZero(1);
    GX_X = Gx * (x_k - x_k_prev_);
    Eigen::VectorXd GX_Y; GX_Y.setZero(1);
    GX_Y = Gx * (y_k - y_k_prev_);

    del_ux(0, 0) = -(px(0) - ref_zmp_(tick,0)) * Gi(0, 0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0, 0) = -(py(0) - ref_zmp_(tick,1)) * Gi(0, 0) - GX_Y(0) - sum_Gd_py_ref;

    UX = UX + del_ux(0, 0);
    UY = UY + del_uy(0, 0);

    x_k = A * x_k + B * UX;
    y_k = A * y_k + B * UY;    

    cp_desired_(0) = x_k(0) + x_k(1) / wn;
    cp_desired_(1) = y_k(0) + y_k(1) / wn; 
}

void CustomController::getComTrajectory_mpc()
{
    if(is_mpc_ctrl_init == true)
    {
        x_mpc_.setZero(); x_mpc_(0) = com_support_init_(0);
        x_mpc_thread3.setZero(); x_mpc_thread3(0) = com_support_init_(0);
        y_mpc_.setZero(); y_mpc_(0) = com_support_init_(1); 
        y_mpc_thread3.setZero(); y_mpc_thread3(0) = com_support_init_(1);

        zx_ref.setZero(mpc_N); 
        zy_ref.setZero(mpc_N); 

        is_mpc_ctrl_init = false;
    }

    pubDataSlowToThread3();
    subDataThread3ToSlow();

    double x_com_lin_spline = (mpc_freq / hz_) * mpc_interpol_cnt_x;
    double y_com_lin_spline = (mpc_freq / hz_) * mpc_interpol_cnt_y;

    com_desired_(0) = x_com_lin_spline * (x_mpc_(0) - x_mpc_prev(0)) + x_mpc_prev(0);
    com_desired_(1) = x_com_lin_spline * (y_mpc_(0) - y_mpc_prev(0)) + y_mpc_prev(0);
    com_desired_(2) = com_height_;

    com_desired_dot_(0) = x_com_lin_spline * (x_mpc_(1) - x_mpc_prev(1)) + x_mpc_prev(1);
    com_desired_dot_(1) = x_com_lin_spline * (y_mpc_(1) - y_mpc_prev(1)) + y_mpc_prev(1);
    com_desired_dot_(2) = 0.0;
    
    cp_desired_ = (com_desired_ + com_desired_dot_ / wn).segment(0, 2);

    mpc_interpol_cnt_x ++;
    mpc_interpol_cnt_y ++;

    if (walking_tick == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1)
    {
        Eigen::Vector3d com_pos_prev;
        Eigen::Vector3d com_pos;
        Eigen::Vector3d com_vel_prev;
        Eigen::Vector3d com_vel;
        Eigen::Vector3d com_acc_prev;
        Eigen::Vector3d com_acc;
        Eigen::Matrix3d temp_rot;
        Eigen::Vector3d temp_pos;

        temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_, 5));
        for (int i = 0; i < 3; i++)
            temp_pos(i) = foot_step_support_frame_(current_step_num_, i);
        
        // CURRENT COM TRAJ
        com_pos_prev(0) = x_mpc_(0);
        com_pos_prev(1) = y_mpc_(0);
        com_pos = temp_rot * (com_pos_prev - temp_pos);

        com_vel_prev(0) = x_mpc_(1);
        com_vel_prev(1) = y_mpc_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = x_mpc_(2);
        com_acc_prev(1) = y_mpc_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        x_mpc_(0) = com_pos(0);
        y_mpc_(0) = com_pos(1);
        x_mpc_(1) = com_vel(0);
        y_mpc_(1) = com_vel(1);
        x_mpc_(2) = com_acc(0);
        y_mpc_(2) = com_acc(1);

        // PREVIOUS COM TRAJ
        com_pos_prev(0) = x_mpc_prev(0);
        com_pos_prev(1) = y_mpc_prev(0);
        com_pos = temp_rot * (com_pos_prev - temp_pos);

        com_vel_prev(0) = x_mpc_prev(1);
        com_vel_prev(1) = y_mpc_prev(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = x_mpc_prev(2);
        com_acc_prev(1) = y_mpc_prev(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        x_mpc_prev(0) = com_pos(0);
        y_mpc_prev(0) = com_pos(1);
        x_mpc_prev(1) = com_vel(0);
        y_mpc_prev(1) = com_vel(1);
        x_mpc_prev(2) = com_acc(0);
        y_mpc_prev(2) = com_acc(1);
    }
}

void CustomController::getFootTrajectory() 
{
    Eigen::Vector6d target_swing_foot; target_swing_foot.setZero();
    target_swing_foot = foot_step_support_frame_.row(current_step_num_).transpose().segment(0,6);

    Eigen::Isometry3d &support_foot_traj           = (is_lfoot_support == true && is_rfoot_support == false) ? lfoot_trajectory_support_ : rfoot_trajectory_support_;
    Eigen::Vector3d &support_foot_traj_euler       = (is_lfoot_support == true && is_rfoot_support == false) ? lfoot_trajectory_euler_support_ : rfoot_trajectory_euler_support_;
    const Eigen::Isometry3d &support_foot_init     = (is_lfoot_support == true && is_rfoot_support == false) ? lfoot_support_init_ : rfoot_support_init_;
    const Eigen::Vector3d &support_foot_euler_init = (is_lfoot_support == true && is_rfoot_support == false) ? lfoot_support_euler_init_ : rfoot_support_euler_init_;

    Eigen::Isometry3d &swing_foot_traj             = (is_lfoot_support == true && is_rfoot_support == false) ? rfoot_trajectory_support_ : lfoot_trajectory_support_;
    Eigen::Vector3d &swing_foot_traj_euler         = (is_lfoot_support == true && is_rfoot_support == false) ? rfoot_trajectory_euler_support_ : lfoot_trajectory_euler_support_;
    const Eigen::Isometry3d &swing_foot_init       = (is_lfoot_support == true && is_rfoot_support == false) ? rfoot_support_init_ : lfoot_support_init_;
    const Eigen::Vector3d &swing_foot_euler_init   = (is_lfoot_support == true && is_rfoot_support == false) ? rfoot_support_euler_init_ : lfoot_support_euler_init_;

    if (is_dsp1 == true)
    {
        support_foot_traj.translation().setZero();
        support_foot_traj_euler.setZero();

        swing_foot_traj.translation() = swing_foot_init.translation();
        swing_foot_traj.translation()(2) = 0.0;
        swing_foot_traj_euler = swing_foot_euler_init;
    }
    else if (is_ssp == true)
    {
        support_foot_traj.translation().setZero();
        support_foot_traj_euler.setZero();

        if (walking_tick < t_start_ + t_dsp1_ + t_ssp_ / 2.0)
        {
            swing_foot_traj.translation()(2) = DyrosMath::cubic(walking_tick, 
                                                                t_start_ + t_dsp1_,
                                                                t_start_ + t_dsp1_ + t_ssp_ / 2.0, 
                                                                0.0, foot_height_, 
                                                                0.0, 0.0);
        }
        else
        {
            swing_foot_traj.translation()(2) = DyrosMath::cubic(walking_tick, 
                                                                t_start_ + t_dsp1_ + t_ssp_ / 2.0,
                                                                t_start_ + t_dsp1_ + t_ssp_, 
                                                                foot_height_, target_swing_foot(2), 
                                                                0.0, 0.0);
        }

        swing_foot_traj.translation().segment(0,2) = DyrosMath::cubicVector<2>(walking_tick, 
                                                                               t_start_ + t_dsp1_,
                                                                               t_start_ + t_dsp1_ + t_ssp_, 
                                                                               swing_foot_init.translation().segment(0,2), target_swing_foot.segment(0,2),
                                                                               Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());

        swing_foot_traj_euler.setZero();
        swing_foot_traj_euler(2) = DyrosMath::cubic(walking_tick, 
                                                    t_start_ + t_dsp1_, 
                                                    t_start_ + t_dsp1_ + t_ssp_, 
                                                    swing_foot_euler_init(2), target_swing_foot(5), 
                                                    0.0, 0.0);
    }
    else if (is_dsp2 == true)
    {
        support_foot_traj_euler.setZero();
        
        swing_foot_traj.translation() = target_swing_foot.segment(0,3);
        swing_foot_traj_euler = target_swing_foot.segment(3,3);
    }

    swing_foot_traj.linear() = DyrosMath::Euler2rot(swing_foot_traj_euler(0), swing_foot_traj_euler(1), swing_foot_traj_euler(2));
    support_foot_traj.linear() = DyrosMath::Euler2rot(support_foot_traj_euler(0), support_foot_traj_euler(1), support_foot_traj_euler(2));
}

void CustomController::getPelvTrajectory()
{
    double z_rot = foot_step_support_frame_(current_step_num_, 5);

    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7 * (com_desired_(0) - com_support_current_(0)); 
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7 * (com_desired_(1) - com_support_current_(1));  
    pelv_trajectory_support_.translation()(2) = pelv_support_current_.translation()(2) + 1.0 * (com_desired_(2) - com_support_current_(2));  

    Eigen::Vector3d pelv_traj_euler; pelv_traj_euler.setZero();

    if (is_dsp1 == true)
    {
        pelv_traj_euler(2) = pelv_support_euler_init_(2);
    }
    else if (is_ssp == true)
    {
        pelv_traj_euler(2) = DyrosMath::cubic(walking_tick, 
                                              t_start_ + t_dsp1_,
                                              t_start_ + t_dsp1_ + t_ssp_, 
                                              pelv_support_euler_init_(2), 
                                              z_rot / 2.0, 
                                              0.0, 0.0);
    }
    else
    {
        pelv_traj_euler(2) = z_rot / 2.0;
    }

    R_angle_input_dot = 2.0 * (0.0 - R_angle) ;
    P_angle_input_dot = 2.0 * (0.0 - P_angle) ;

    R_angle_input = DyrosMath::minmax_cut(R_angle_input + R_angle_input_dot * del_t, -5.0 * DEG2RAD, 5.0 * DEG2RAD);
    P_angle_input = DyrosMath::minmax_cut(P_angle_input + P_angle_input_dot * del_t, -5.0 * DEG2RAD, 5.0 * DEG2RAD);

    pelv_traj_euler(0) = R_angle_input;
    pelv_traj_euler(1) = P_angle_input;    
    
    pelv_trajectory_support_.linear() = DyrosMath::Euler2rot(pelv_traj_euler(0), pelv_traj_euler(1), pelv_traj_euler(2));
}

void CustomController::computeIkControl(const Eigen::Isometry3d &float_trunk_transform, const Eigen::Isometry3d &float_lleg_transform, const Eigen::Isometry3d &float_rleg_transform, Eigen::Vector12d &q_des)
{
    Eigen::Vector3d R_r, R_D, L_r, L_D;

    L_D << 0.11, +0.1025, -0.1025;
    R_D << 0.11, -0.1025, -0.1025;

    L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * L_D - float_lleg_transform.translation());
    R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * R_D - float_rleg_transform.translation());

    double R_C = 0, L_C = 0, L_upper = 0.351, L_lower = 0.351, R_alpha = 0, L_alpha = 0;

    L_C = sqrt(pow(L_r(0), 2) + pow(L_r(1), 2) + pow(L_r(2), 2));
    R_C = sqrt(pow(R_r(0), 2) + pow(R_r(1), 2) + pow(R_r(2), 2));
     
    double knee_acos_var_L = 0;
    double knee_acos_var_R = 0;

    knee_acos_var_L = (pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2))/ (2 * L_upper * L_lower);
    knee_acos_var_R = (pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2))/ (2 * L_upper * L_lower);

    knee_acos_var_L = DyrosMath::minmax_cut(knee_acos_var_L, -0.99, + 0.99);
    knee_acos_var_R = DyrosMath::minmax_cut(knee_acos_var_R, -0.99, + 0.99);

    q_des(3) = (-acos(knee_acos_var_L) + M_PI);  
    q_des(9) = (-acos(knee_acos_var_R) + M_PI);
    
    L_alpha = asin(L_upper / L_C * sin(M_PI - q_des(3)));
    R_alpha = asin(L_upper / R_C * sin(M_PI - q_des(9)));
    
    q_des(4) = -atan2(L_r(0), sqrt(pow(L_r(1), 2) + pow(L_r(2), 2))) - L_alpha;
    q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1), 2) + pow(R_r(2), 2))) - R_alpha;

    Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
    Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
    Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

    L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3) - q_des(4));
    L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
    R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9) - q_des(10));
    R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11));

    L_Hip_rot_mat.setZero();
    R_Hip_rot_mat.setZero();

    L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat;
    R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

    q_des(0) = atan2(-L_Hip_rot_mat(0, 1), L_Hip_rot_mat(1, 1));                                                       // Hip yaw
    q_des(1) = atan2(L_Hip_rot_mat(2, 1), -L_Hip_rot_mat(0, 1) * sin(q_des(0)) + L_Hip_rot_mat(1, 1) * cos(q_des(0))); // Hip roll
    q_des(2) = atan2(-L_Hip_rot_mat(2, 0), L_Hip_rot_mat(2, 2));                                                       // Hip pitch
    q_des(3) = q_des(3);                                                                                               // Knee pitch
    q_des(4) = q_des(4);                                                                                               // Ankle pitch
    q_des(5) = atan2(L_r(1), L_r(2));                                                                                  // Ankle roll

    q_des(6) = atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1));
    q_des(7) = atan2(R_Hip_rot_mat(2, 1), -R_Hip_rot_mat(0, 1) * sin(q_des(6)) + R_Hip_rot_mat(1, 1) * cos(q_des(6)));
    q_des(8) = atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2));
    q_des(9) = q_des(9);
    q_des(10) = q_des(10);
    q_des(11) = atan2(R_r(1), R_r(2));
}

void CustomController::contactWrenchCalculator()
{ 
    ////// DEL ZMP CALCULATION //////
    del_zmp = kp_cp * (cp_measured_ - cp_desired_);

    ////// CONTACT WRENCH CALCULATION //////
    double alpha = 0;
    double F_R = 0, F_L = 0;
    double Tau_all_y, Tau_R_y, Tau_L_y = 0;
    double Tau_all_x, Tau_R_x, Tau_L_x = 0; 
     
    alpha = (ZMP_Y_REF_alpha_ + del_zmp(1) - rfoot_support_current_.translation()(1)) / (lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
    alpha = DyrosMath::minmax_cut(alpha, 0.0, 1.0);
    alpha_lpf_ = DyrosMath::lpf(alpha, alpha_lpf_, 2000.0, 50.0);
    alpha_lpf_ = DyrosMath::minmax_cut(alpha_lpf_, 0.0, 1.0);

    //////////// FORCE ////////////
    F_R = -(1 - alpha_lpf_) * rd_.link_[COM_id].mass * GRAVITY;
    F_L =     - alpha_lpf_  * rd_.link_[COM_id].mass * GRAVITY;

    //////////// TORQUE ////////////
    Tau_all_x = -((rfoot_support_current_.translation()(1) - (ZMP_Y_REF_ + del_zmp(1))) * F_R + (lfoot_support_current_.translation()(1) - (ZMP_Y_REF_ + del_zmp(1))) * F_L);
    Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_L);
 
    Tau_R_x = (1 - alpha_lpf_) * Tau_all_x;
    Tau_R_y =-(1 - alpha_lpf_) * Tau_all_y;
    Tau_L_x = alpha_lpf_ * Tau_all_x;
    Tau_L_y =-alpha_lpf_ * Tau_all_y;

    lfoot_contact_wrench << 0.0, 0.0, F_L, Tau_L_x, Tau_L_y, 0.0;
    rfoot_contact_wrench << 0.0, 0.0, F_R, Tau_R_x, Tau_R_y, 0.0;
  
    contact_wrench_torque.setZero();
    contact_wrench_torque = rd_.link_[Left_Foot].Jac().rightCols(MODEL_DOF).transpose()  * lfoot_contact_wrench + rd_.link_[Right_Foot].Jac().rightCols(MODEL_DOF).transpose() * rfoot_contact_wrench;
}

void CustomController::supportToFloatPattern()
{
    pelv_trajectory_float_  = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * pelv_trajectory_support_;
    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * lfoot_trajectory_support_;
    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * rfoot_trajectory_support_;
}

void CustomController::updateNextStepTime()
{       
    if (walking_tick >= t_last_)
    {   
        if (current_step_num_ != total_step_num_ - 1)
        {   
            // t_total_ = t_total_ + 0.05*hz_;
            t_start_ = t_last_ + 1;
            t_start_real_ = t_start_ + t_rest_init_;
            t_last_ = t_start_ + t_total_ - 1;
            current_step_num_++;            
        }  

        is_support_foot_change = true;
    }

    if ((current_step_num_ == total_step_num_ - 1 && walking_tick >= t_total_ + t_last_) != true)
    {
        walking_tick++;
    }
}

void CustomController::walkingStateMachine()
{
    if (foot_step_(current_step_num_, 6) == 1) 
    {
        is_lfoot_support = true;
        is_rfoot_support = false;
    }
    else if (foot_step_(current_step_num_, 6) == 0) 
    {
        is_lfoot_support = false;
        is_rfoot_support = true;
    }

    if (walking_tick < t_start_ + t_rest_init_ + t_double1_)
    {
        is_dsp1 = true;
        is_ssp  = false;
        is_dsp2 = false;
    }
    else if (walking_tick >= t_start_ + t_rest_init_ + t_double1_ && walking_tick < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        is_dsp1 = false;
        is_ssp  = true;
        is_dsp2 = false;
    }
    else
    {
        is_dsp1 = false;
        is_ssp  = false;
        is_dsp2 = true;
    } 
}

void CustomController::pubDataSlowToThread3()
{
    if(atb_mpc_update_ == false)
    {
        atb_mpc_update_ = true;

        walking_tick_container = walking_tick;
        zmp_start_time_container = zmp_start_time_; 
        current_step_container = current_step_num_;
        ref_zmp_container = ref_zmp_;

        x_mpc_container = x_mpc_;
        y_mpc_container = y_mpc_;

        atb_mpc_update_ = false;
    }
}

void CustomController::subDataSlowToThread3()
{
    if(atb_mpc_update_ == false)
    {
        atb_mpc_update_ = true;

        walking_tick_thread3 = walking_tick_container;
        zmp_start_time_thread3 = zmp_start_time_container;
        current_step_thread3 = current_step_container;

        ref_zmp_thread3 = ref_zmp_container;

        x_mpc_thread3 = x_mpc_container;
        y_mpc_thread3 = y_mpc_container;

        atb_mpc_update_ = false;
    }
}

void CustomController::pubDataThread3ToSlow()
{
    if (atb_mpc_x_update_ == false)
    {
        atb_mpc_x_update_ = true;
        
        x_mpc_container2 = x_mpc_thread3;

        current_step_checker = current_step_thread3;

        atb_mpc_x_update_ = false;
    }

    is_mpc_x_update = true;

    if (atb_mpc_y_update_ == false)
    {
        atb_mpc_y_update_ = true;
        
        y_mpc_container2 = y_mpc_thread3;

        current_step_checker = current_step_thread3;

        atb_mpc_y_update_ = false;
    }

    is_mpc_y_update = true;
}

void CustomController::subDataThread3ToSlow()
{
    if (is_mpc_x_update == true)
    {
        if (atb_mpc_x_update_ == false)
        {
            atb_mpc_x_update_ = true;
            
            x_mpc_prev = x_mpc_;
            
            if (current_step_checker == current_step_num_)
            {
                x_mpc_ = x_mpc_container2;

                mpc_interpol_cnt_x = 1;
            }
            else
            {
                std::cout << "MPC output X is ignored: step number mismatch (MPC step = " 
                        << current_step_checker << ", real-time step = " 
                        << current_step_num_ << ")." << std::endl;
            }

            atb_mpc_x_update_ = false;
        }
        
        is_mpc_x_update = false;
    }

    if (is_mpc_y_update == true)
    {
        if (atb_mpc_y_update_ == false)
        {
            atb_mpc_y_update_ = true;

            y_mpc_prev = y_mpc_;

            if (current_step_checker == current_step_num_)
            {
                y_mpc_ = y_mpc_container2;

                mpc_interpol_cnt_y = 1;
            }
            else
            {
                std::cout << "MPC output Y is ignored: step number mismatch (MPC step = " 
                        << current_step_checker << ", real-time step = " 
                        << current_step_num_ << ")." << std::endl;
            }

            atb_mpc_y_update_ = false;
        }

        is_mpc_y_update = false;
    }
}