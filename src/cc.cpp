#include "cc.h"

using namespace TOCABI;

ofstream dataCC1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC1.txt");
ofstream dataCC2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC2.txt");

CustomController::CustomController(RobotData &rd) : rd_(rd), kin_wbc_(MODEL_DOF_VIRTUAL), dyn_wbc_(MODEL_DOF_VIRTUAL)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    ControlVal_.setZero();

    // Load Robot Model
    std::string urdf_path, desc_package_path;
    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);

    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_, true, false);

    // joy_sub_ = nh_cc_.subscribe<sensor_msgs::Joy>("/joy", 10, &CustomController::joyCallback, this);
    xbox_joy_sub_ = nh_cc_.subscribe<sensor_msgs::Joy>("/joy", 10, &CustomController::xBoxJoyCallback, this);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

void CustomController::computeSlow()
{
    queue_cc_.callAvailable(ros::WallDuration());

    
    if (rd_.tc_.mode == 6)
    {   
        if (is_mode_6_init == true)
        {
            loadParams();

            q_init_ = rd_.q_;

            cout << "COMPUTESLOW MODE 6 IS NOW INITIALIZED" << endl;
            cout << "TIME: "<< rd_.control_time_ << endl; 

            is_mode_6_init = false;
        }

        moveInitialPose();

        rd_.torque_desired = (Kp_diag * (rd_.q_desired - rd_.q_)) - (Kd_diag * rd_.q_dot_);
    }
    else if (rd_.tc_.mode == 7)
    {
        stateManager();

        if(is_mode_7_init == true)
        {
            saveInitialState();

            dyn_wbc_.setRobotSystemParameters(0.8,                 // Friction Coefficient
                                              0.3,                 // Foot size
                                              0.16,                // Foot width
                                              2,                   // Contact Dim
                                              rd_.torque_limit,    // Torque limit
                                              joint_pos_limit_l_,
                                              joint_pos_limit_h_,
                                              joint_vel_limit_l_,
                                              joint_vel_limit_h_,
                                              1500.0,              // Max vertical contact force
                                              100.0);                // Min vertical contact force

            Eigen::VectorVQd W_Q;      W_Q      = 2000.0 * Eigen::VectorVQd::Ones();
            Eigen::VectorQd  W_torque; W_torque = 1.0 * Eigen::VectorQd::Ones();
            Eigen::Vector12d W_lambda; W_lambda = 1e-3 * Eigen::Vector12d::Ones();
            Eigen::VectorQd  W_torque_prev; W_torque_prev = 2000.0 * Eigen::VectorQd::Ones();
            dyn_wbc_.setWbcWeights(W_Q, W_torque, W_lambda, W_torque_prev);

            cout << "COMPUTESLOW MODE 7 IS NOW INITIALIZED" << endl;
            cout << "TIME: "<< rd_.control_time_ << endl; 

            is_mode_7_init = false;
        }

        motion_mode_ = TestMotionType::PelvHandJoy;
        runTestMotion(1.0, 0.05, 0.2);

        kin_wbc_.computeTaskSpaceKinematicWBC(task_hierarchy,
                                              x_desired, dx_desired, ddx_desired,
                                              R_desired, w_desired, dw_desired,
                                              task_Kp, task_Kv, 
                                              base_ee_pos, base_ee_rot,
                                              base_ee_v, base_ee_w,
                                              base_Jac_v, base_Jac_w,
                                              base_contact_Jac,
                                              M_inv_, rd_.q_dot_virtual_,
                                              dq_des, qdot_des, qddot_des);

        rd_.q_desired += qdot_des.segment(6, MODEL_DOF) / hz_;

        Eigen::VectorQd torque_unbound; torque_unbound.setZero();
        dyn_wbc_.getRobotStates(M_inv_, G_, base_contact_Jac, qddot_des, rd_.q_, rd_.q_dot_);
        torque_unbound = dyn_wbc_.computeDynamicWBC(); 
        // torque_unbound = dyn_wbc_.computeDynamicWBC() + Kp_diag * (rd_.q_desired - rd_.q_) + Kd_diag * (qdot_des.segment(6, MODEL_DOF) - rd_.q_dot_);  

        dataCC1 << dx_desired[base_link_name].transpose() << " " << base_ee_v[base_link_name].transpose() << std::endl;
        dataCC2 << w_desired[base_link_name].transpose() << " " << base_ee_w[base_link_name].transpose() << std::endl;


        //--- Final Torque Command
        Eigen::VectorQd torque_bound;   torque_bound.setZero();
        for (int i = 0; i < MODEL_DOF; i++) {
            torque_bound(i) = DyrosMath::minmax_cut(torque_unbound(i), -rd_.torque_limit(i), rd_.torque_limit(i));
        }
        rd_.torque_desired = torque_bound;
    }
}

void CustomController::computeFast()
{
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::loadParams()
{
    Kp.setZero(MODEL_DOF);          
    Kd.setZero(MODEL_DOF);
    Kp_diag.setZero(MODEL_DOF, MODEL_DOF);
    Kd_diag.setZero(MODEL_DOF, MODEL_DOF);

    std::vector<double> kp_vec, kd_vec;
    std::vector<double> pos_low_deg, pos_high_deg;
    std::vector<double> vel_low, vel_high;

    nh_cc_.getParam("/tocabi_controller/joint_gains/Kp", kp_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Kd", kd_vec);
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

    Kp_diag = Kp.asDiagonal();
    Kd_diag = Kd.asDiagonal();

    // Position Limits (convert deg to rad)
    for (int i = 0; i < MODEL_DOF; ++i)
    {
        joint_pos_limit_l_(i) = pos_low_deg[i] * DEG2RAD;
        joint_pos_limit_h_(i) = pos_high_deg[i] * DEG2RAD;
    }

    // Velocity Limits
    for (int i = 0; i < MODEL_DOF; ++i)
    {
        joint_vel_limit_l_(i) = vel_low[i];
        joint_vel_limit_h_(i) = vel_high[i];
    }

    // Task Gain
    task_Kp[base_link_name]  = 400.0 * Eigen::Vector3d::Ones();
    task_Kp[chest_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kp[head_link_name]  = 400.0 * Eigen::Vector3d::Ones();
    task_Kp[lfoot_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kp[rfoot_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kp[lhand_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kp[rhand_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kp[com_name] = 400.0 * Eigen::Vector3d::Ones();

    task_Kv[base_link_name]  = 40.0 * Eigen::Vector3d::Ones();
    task_Kv[chest_link_name] = 40.0 * Eigen::Vector3d::Ones();
    task_Kv[head_link_name]  = 40.0 * Eigen::Vector3d::Ones();
    task_Kv[lfoot_link_name] = 40.0 * Eigen::Vector3d::Ones();
    task_Kv[rfoot_link_name] = 40.0 * Eigen::Vector3d::Ones();
    task_Kv[lhand_link_name] = 40.0 * Eigen::Vector3d::Ones();
    task_Kv[rhand_link_name] = 40.0 * Eigen::Vector3d::Ones();
    task_Kv[com_name] = 40.0 * Eigen::Vector3d::Ones();
}

void CustomController::moveInitialPose()
{
    static int initial_tick = 0;

    q_init_des; q_init_des.setZero();
    q_init_des = q_init_;
    
    q_init_des(15) = 0.0;
    q_init_des(16) = -0.3;
    q_init_des(17) = 1.57;
    q_init_des(18) = -1.2;
    q_init_des(19) = -1.57; // elbow
    q_init_des(20) = 1.5;
    q_init_des(21) = 0.4;
    q_init_des(22) = -0.2;

    q_init_des(23) = 0; // yaw
    q_init_des(24) = 0.3; // pitch

    q_init_des(25) = 0.0;
    q_init_des(26) = 0.3;
    q_init_des(27) = -1.57;
    q_init_des(28) = 1.2;
    q_init_des(29) = 1.57; // elbow
    q_init_des(30) = -1.5;
    q_init_des(31) = -0.4;
    q_init_des(32) = 0.2;

    rd_.q_desired = DyrosMath::cubicVector<MODEL_DOF>(initial_tick, 0, 2.0 * hz_, q_init_, q_init_des, Eigen::VectorQd::Zero(), Eigen::VectorQd::Zero()); 

    initial_tick++;
}

void CustomController::stateManager()
{
    Eigen::Vector3d base_pos = rd_.link_[Pelvis].xpos; 
    Eigen::Matrix3d base_rot = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(2)); 
    contact_Jac.setZero(12, MODEL_DOF_VIRTUAL);
    base_contact_Jac.setZero(12, MODEL_DOF_VIRTUAL);



    for (const auto& [name, idx] : link_index_map)
    {
        //--- Global frame
        Jac_v[name] = rd_.link_[idx].Jac().topRows(3);
        Jac_w[name] = rd_.link_[idx].Jac().bottomRows(3);
        ee_pos[name] = rd_.link_[idx].xpos;
        ee_rot[name] = rd_.link_[idx].rotm;
        ee_v[name] = rd_.link_[idx].v;
        ee_w[name] = rd_.link_[idx].w;
        contact_Jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = Jac_v[lfoot_link_name]; 
        contact_Jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = Jac_w[lfoot_link_name]; 
        contact_Jac.block(6, 0, 3, MODEL_DOF_VIRTUAL) = Jac_v[rfoot_link_name]; 
        contact_Jac.block(9, 0, 3, MODEL_DOF_VIRTUAL) = Jac_w[rfoot_link_name]; 

        //--- Base frame
        base_Jac_v[name]  = base_rot.transpose() * Jac_v[name];
        base_Jac_w[name]  = base_rot.transpose() * Jac_w[name];
        base_ee_pos[name] = base_rot.transpose() * (ee_pos[name] - base_pos);
        base_ee_rot[name] = base_rot.transpose() *  ee_rot[name];                             
        base_ee_v[name]   = base_rot.transpose() *  ee_v[name];                               
        base_ee_w[name]   = base_rot.transpose() *  ee_w[name]; 
        base_contact_Jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = base_Jac_v[lfoot_link_name]; 
        base_contact_Jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = base_Jac_w[lfoot_link_name]; 
        base_contact_Jac.block(6, 0, 3, MODEL_DOF_VIRTUAL) = base_Jac_v[rfoot_link_name]; 
        base_contact_Jac.block(9, 0, 3, MODEL_DOF_VIRTUAL) = base_Jac_w[rfoot_link_name];

    }

    Eigen::VectorQVQd base_q_virtual_;
    base_q_virtual_.segment(0,3) = base_ee_pos[base_link_name];
    
    Quaterniond base_quat(base_ee_rot[base_link_name]);
    base_quat.normalize();
    
    base_q_virtual_(3)  = base_quat.x();
    base_q_virtual_(4)  = base_quat.y();
    base_q_virtual_(5)  = base_quat.z();
    base_q_virtual_(39) = base_quat.w();
    
    base_q_virtual_.segment(6, MODEL_DOF) = rd_.q_;

    //--- Dynamics
    M_.setZero(); G_.setZero(); M_inv_.setZero();
    M_temp_.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL); G_temp_.setZero(MODEL_DOF_VIRTUAL);

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, base_q_virtual_, M_temp_, true);
    M_ = M_temp_;
    // M_inv_ = M_.inverse();
    M_inv_ = M_.llt().solve(MatrixXd::Identity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL));
    
    RigidBodyDynamics::NonlinearEffects(model_, base_q_virtual_, Eigen::VectorXd::Zero(MODEL_DOF_QVIRTUAL), G_temp_);
    G_ = G_temp_;

    for (const auto& [name, idx] : link_index_map)
    {
        base_Lambda_v[name] = (base_Jac_v[name] * M_ * base_Jac_v[name].transpose()).inverse();
        base_Lambda_w[name] = (base_Jac_w[name] * M_ * base_Jac_w[name].transpose()).inverse();
    }
}

void CustomController::saveInitialState()
{
    init_ee_pos = ee_pos;
    init_ee_rot = ee_rot;
    init_ee_v = ee_v;
    init_ee_w = ee_w;

    init_base_ee_pos = base_ee_pos;
    init_base_ee_rot = base_ee_rot;
    init_base_ee_v = base_ee_v;
    init_base_ee_w = base_ee_w;

    rd_.q_desired = q_init_des;

    for (const auto& [name, idx] : link_index_map)
    {
        x_desired[name]   = init_base_ee_pos[name];
        dx_desired[name]  = Eigen::Vector3d::Zero();
        ddx_desired[name] = Eigen::Vector3d::Zero();

        R_desired[name]   = init_base_ee_rot[name];
        w_desired[name]   = Eigen::Vector3d::Zero();
        dw_desired[name]  = Eigen::Vector3d::Zero();
    }

    q_des.segment(6, MODEL_DOF)= q_init_des;
    dq_des.setZero(); 
    qdot_des.setZero();
    qddot_des.setZero(); 
}

void CustomController::runTestMotion(double traj_time, double pelv_dist, double hand_dist)
{
    switch (motion_mode_)
    {
        case TestMotionType::Pelv:
            movePelvPose(traj_time, pelv_dist);
            break;
        case TestMotionType::Hand:
            moveHandPose(traj_time, hand_dist);
            break;
        case TestMotionType::PelvHand:
            movePelvHandPose(traj_time, pelv_dist, hand_dist);
            break;
        case TestMotionType::PelvJoy:
            movePelvPoseJoy(target_vel_x_, target_vel_y_, target_vel_yaw_);
        case TestMotionType::PelvHandJoy:
            movePelvHandPoseJoy(target_vel_x_, target_vel_y_, target_vel_yaw_, traj_time, hand_dist);
            break;
        case TestMotionType::None:
        default:
            break;
    }
}

void CustomController::movePelvPose(double traj_time, double pelv_dist)
{
    task_hierarchy= {
        { {base_link_name, TaskType::Position}, {base_link_name, TaskType::Orientation} },
    };

    static int tick = 0;

    x_desired[base_link_name] = init_ee_pos[base_link_name];
    dx_desired[base_link_name].setZero();
    ddx_desired[base_link_name].setZero();
    x_desired[base_link_name](1) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                        init_ee_pos[base_link_name](1), 
                                                        init_ee_pos[base_link_name](1) + pelv_dist, 
                                                        0.0, 0.0);

    dx_desired[base_link_name](1) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                        init_ee_pos[base_link_name](1), 
                                                        init_ee_pos[base_link_name](1) + pelv_dist, 
                                                        0.0, 0.0);

    ddx_desired[base_link_name](1) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                            init_ee_pos[base_link_name](1), 
                                                            init_ee_pos[base_link_name](1) + pelv_dist, 
                                                            0.0, 0.0);

    x_desired[base_link_name]   = ee_rot[base_link_name].transpose() * (x_desired[base_link_name] - ee_pos[base_link_name]);
    dx_desired[base_link_name]  = ee_rot[base_link_name].transpose() * (dx_desired[base_link_name]);
    ddx_desired[base_link_name] = ee_rot[base_link_name].transpose() * (ddx_desired[base_link_name]);
    
    tick++;
}

void CustomController::moveHandPose(double traj_time,  double hand_dist)
{
    task_hierarchy= {
            { {base_link_name, TaskType::Position}, {base_link_name, TaskType::Orientation} },
            // { {chest_link_name, TaskType::Orientation} },
            // { {head_link_name, TaskType::Orientation} },
            { {lhand_link_name, TaskType::Position}, {lhand_link_name, TaskType::Orientation}, {rhand_link_name, TaskType::Position}, {rhand_link_name, TaskType::Orientation} },
            { {com_name, TaskType::Position}}
    };

    static int tick = 0;

    //--- Hand Test
    x_desired[lhand_link_name](2) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                        init_base_ee_pos[lhand_link_name](2), 
                                                        init_base_ee_pos[lhand_link_name](2) + hand_dist, 
                                                        0.0, 0.0);

    dx_desired[lhand_link_name](2) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                            init_base_ee_pos[lhand_link_name](2), 
                                                            init_base_ee_pos[lhand_link_name](2) + hand_dist, 
                                                            0.0, 0.0);

    ddx_desired[lhand_link_name](2) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                            init_base_ee_pos[lhand_link_name](2), 
                                                            init_base_ee_pos[lhand_link_name](2) + hand_dist, 
                                                            0.0, 0.0);

    x_desired[rhand_link_name](2) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                        init_base_ee_pos[rhand_link_name](2), 
                                                        init_base_ee_pos[rhand_link_name](2) - hand_dist, 
                                                        0.0, 0.0);

    dx_desired[rhand_link_name](2) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                            init_base_ee_pos[rhand_link_name](2), 
                                                            init_base_ee_pos[rhand_link_name](2) - hand_dist, 
                                                            0.0, 0.0);

    ddx_desired[rhand_link_name](2) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                            init_base_ee_pos[rhand_link_name](2), 
                                                            init_base_ee_pos[rhand_link_name](2) - hand_dist, 
                                                            0.0, 0.0);
    
    tick++;
}

void CustomController::movePelvHandPose(double traj_time, double pelv_dist, double hand_dist)
{
    task_hierarchy= {
            { {base_link_name, TaskType::Position}, {base_link_name, TaskType::Orientation} },
            { {chest_link_name, TaskType::Orientation} },
            { {head_link_name, TaskType::Orientation} },
            { {lhand_link_name, TaskType::Position}, {lhand_link_name, TaskType::Orientation}, {rhand_link_name, TaskType::Position}, {rhand_link_name, TaskType::Orientation} }
    };

    static int tick = 0;

    //--- Pevis Test
    x_desired[base_link_name]   = init_ee_pos[base_link_name];
    dx_desired[base_link_name].setZero();
    ddx_desired[base_link_name].setZero();
    x_desired[base_link_name](1) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                   init_ee_pos[base_link_name](1), 
                                                   init_ee_pos[base_link_name](1) + pelv_dist, 
                                                   0.0, 0.0);

    dx_desired[base_link_name](1) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                       init_ee_pos[base_link_name](1), 
                                                       init_ee_pos[base_link_name](1) + pelv_dist, 
                                                       0.0, 0.0);

    ddx_desired[base_link_name](1) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                         init_ee_pos[base_link_name](1), 
                                                         init_ee_pos[base_link_name](1) + pelv_dist, 
                                                         0.0, 0.0);

    x_desired[base_link_name]   = ee_rot[base_link_name].transpose() * (x_desired[base_link_name] - ee_pos[base_link_name]);
    dx_desired[base_link_name]  = ee_rot[base_link_name].transpose() * (dx_desired[base_link_name]);
    ddx_desired[base_link_name] = ee_rot[base_link_name].transpose() * (ddx_desired[base_link_name]);
}

void CustomController::movePelvPoseJoy(const double& vx, const double& vy, const double& wz)
{
    // task_hierarchy= {
    //     { {base_link_name, TaskType::Position}, {base_link_name, TaskType::Orientation} },
    // };
    task_hierarchy= {
            { {base_link_name, TaskType::Position}, {base_link_name, TaskType::Orientation} },
            { {chest_link_name, TaskType::Orientation} },
            { {head_link_name, TaskType::Orientation} },
            { {lhand_link_name, TaskType::Position}, {lhand_link_name, TaskType::Orientation}, {rhand_link_name, TaskType::Position}, {rhand_link_name, TaskType::Orientation} }
    };

    //--- Translation
    x_desired[base_link_name](0) = vx / hz_;
    x_desired[base_link_name](1) = vy / hz_;

    dx_desired[base_link_name](0) = vx;
    dx_desired[base_link_name](1) = vy;

    //--- Orientation
    w_desired[base_link_name](2) = wz;

    Eigen::Vector3d eulerDot_desired = AngvelToEulerRates(w_desired[base_link_name], DyrosMath::rot2Euler(base_ee_rot[base_link_name]));
    Eigen::Vector3d euler_desired = eulerDot_desired / hz_;
    R_desired[base_link_name] = DyrosMath::Euler2rot(euler_desired(0), euler_desired(1), euler_desired(2));
}

void CustomController::movePelvHandPoseJoy(const double& vx, const double& vy, const double& wz, const double& traj_time, const double& hand_dist)
{
    task_hierarchy= {
            { {base_link_name, TaskType::Position}, {base_link_name, TaskType::Orientation} },
            { {chest_link_name, TaskType::Orientation} },
            { {head_link_name, TaskType::Orientation} },
            { {lhand_link_name, TaskType::Position}, {lhand_link_name, TaskType::Orientation}, {rhand_link_name, TaskType::Position}, {rhand_link_name, TaskType::Orientation} }
    };

    static int tick = 0;

    //--- Pelvis Translation
    x_desired[base_link_name](0) = vx / hz_;
    x_desired[base_link_name](1) = vy / hz_;

    dx_desired[base_link_name](0) = vx;
    dx_desired[base_link_name](1) = vy;

    //--- Pelvis Orientation
    w_desired[base_link_name](2) = wz;

    Eigen::Vector3d eulerDot_desired = AngvelToEulerRates(w_desired[base_link_name], DyrosMath::rot2Euler(base_ee_rot[base_link_name]));
    Eigen::Vector3d euler_desired = eulerDot_desired / hz_;
    R_desired[base_link_name] = DyrosMath::Euler2rot(euler_desired(0), euler_desired(1), euler_desired(2));

    //--- Hand Test
    x_desired[lhand_link_name](2) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                     init_base_ee_pos[lhand_link_name](2), 
                                                     init_base_ee_pos[lhand_link_name](2) + hand_dist, 
                                                     0.0, 0.0);

    dx_desired[lhand_link_name](2) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                         init_base_ee_pos[lhand_link_name](2), 
                                                         init_base_ee_pos[lhand_link_name](2) + hand_dist, 
                                                         0.0, 0.0);

    ddx_desired[lhand_link_name](2) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                           init_base_ee_pos[lhand_link_name](2), 
                                                           init_base_ee_pos[lhand_link_name](2) + hand_dist, 
                                                           0.0, 0.0);

    x_desired[rhand_link_name](2) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                     init_base_ee_pos[rhand_link_name](2), 
                                                     init_base_ee_pos[rhand_link_name](2) - hand_dist, 
                                                     0.0, 0.0);

    dx_desired[rhand_link_name](2) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                         init_base_ee_pos[rhand_link_name](2), 
                                                         init_base_ee_pos[rhand_link_name](2) - hand_dist, 
                                                         0.0, 0.0);

    ddx_desired[rhand_link_name](2) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                           init_base_ee_pos[rhand_link_name](2), 
                                                           init_base_ee_pos[rhand_link_name](2) - hand_dist, 
                                                           0.0, 0.0);

    tick++;
}

//--- Joy Utils
void CustomController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    target_vel_x_ = DyrosMath::minmax_cut(joy->axes[0]*0.5, -0.5, 0.5);
    target_vel_y_ = 0.0; // DyrosMath::minmax_cut(joy->axes[1], -0.0, 0.0);
    target_vel_yaw_ = -DyrosMath::minmax_cut(joy->axes[2]*0.3, -0.3, 0.3);
}

void CustomController::xBoxJoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double vel_threshold = 0.2;

    target_vel_x_   = DyrosMath::minmax_cut(joy->axes[1] * vel_threshold, -vel_threshold, vel_threshold);
    target_vel_y_   = DyrosMath::minmax_cut(joy->axes[0] * vel_threshold, -vel_threshold, vel_threshold);
    target_vel_yaw_ = DyrosMath::minmax_cut(joy->axes[3] * vel_threshold, -vel_threshold, vel_threshold);
}