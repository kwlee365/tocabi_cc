#include "cc.h"

using namespace TOCABI;

ofstream dataCC1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC1.txt");
ofstream dataCC2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC2.txt");
ofstream dataCC3("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC3.txt");
ofstream dataCC4("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC4.txt");
ofstream dataCC5("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC5.txt");
ofstream dataCC6("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC6.txt");

CustomController::CustomController(RobotData &rd) : rd_(rd), kin_wbc_(MODEL_DOF_VIRTUAL),  dyn_wbc_(MODEL_DOF_VIRTUAL)
{
    //--- ROS Node Handle
    nh_cc_.setCallbackQueue(&queue_cc_);
    ControlVal_.setZero();

    //--- Load Robot Model
    std::string urdf_path, desc_package_path;
    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_, true, false);

    //--- Joy Callback
    xbox_joy_sub_ = nh_cc_.subscribe<sensor_msgs::Joy>("/joy", 10, &CustomController::xBoxJoyCallback, this);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

void CustomController::computeSlow()
{
    queue_cc_.callAvailable(ros::WallDuration());
    int where_am_i = 0;
    
    if (rd_.tc_.mode == 6)
    {   
        if (is_mode_6_init == true)
        {
            loadParams();

            q_init_ = rd_.q_;
            qdot_LPF.setZero();

            WBC::SetContact(rd_, true, true);
            
            cout << "COMPUTESLOW MODE 6 IS NOW INITIALIZED" << endl;
            cout << "TIME: "<< rd_.control_time_ << endl; 

            is_mode_6_init = false;
        }

        moveInitialPose();

        rd_.torque_desired = (Kp_diag * (rd_.q_desired - rd_.q_)) - (Kd_diag * rd_.q_dot_);
    }
    else if (rd_.tc_.mode == 7)
    {
        where_am_i = 1;
        if(is_mode_7_working == true)
        {
            stateManager();

            if(is_mode_7_init == true)
            {
                saveInitialState();
                dyn_wbc_.setRobotSystemParameters(0.8,                  // Friction Coefficient
                                                  0.3,                  // Foot size
                                                  0.16);                // Foot width

                        
                cout << "COMPUTESLOW MODE 7 IS NOW INITIALIZED" << endl;
                cout << "TIME: "<< rd_.control_time_ << endl; 

                is_mode_7_init = false;
            }

            contactStateManager();

            motion_mode_ = TestMotionType::PelvHand;
            // runTestMotion(2.0, 0.05, 0.0, 0.2, 0.6); // PelvHand (sine)
            runTestMotion(5.0, 0.10, 0.5, 0.2, 0.6); // PelvHand
            // runTestMotion(5.0, 0.12, 0.5, 0.2, 0.6);    // Taichi

            //--- Whole-body Inverse Kinematics
            kin_wbc_.computeTaskSpaceKinematicWBC(task_hierarchy,
                                                  x_desired, dx_desired, ddx_desired,
                                                  R_desired, w_desired, dw_desired,
                                                  task_pos_Kp, task_ori_Kp, 
                                                  base_ee_pos, base_ee_rot,
                                                  base_ee_v, base_ee_w,
                                                  base_Jac_v, base_Jac_w, base_contact_Jac, 
                                                  qdot_, qdot_des);
            kin_wbc_.safetyFilter(qdot_des, q_, joint_pos_limit_l_, joint_pos_limit_h_, joint_vel_limit_l_, joint_vel_limit_h_);

            q_des += qdot_des / hz_;
            rd_.q_desired = q_des.tail(MODEL_DOF);

            qddot_des.setZero();
            qddot_des =  Kd_virtual_diag * (qdot_des - qdot_) + Kp_virtual_diag * (q_des - q_);
            qddot_des.segment(3,3) =  Kd_virtual_diag.block(3,3,3,3) * (qdot_des.segment(3,3) - qdot_.segment(3,3)) 
                                    - Kp_virtual_diag.block(3,3,3,3) * DyrosMath::getPhi(DyrosMath::Euler2rot(q_(3), q_(4), q_(5)), DyrosMath::Euler2rot(q_des(3), q_des(4), q_des(5)));

            //--- Contact Constrained Whole-body Control
            dyn_wbc_.updateContactState(contact_mode_);
            dyn_wbc_.getRobotStates(q_,
                                    qdot_,
                                    qddot_des,
                                    M_,
                                    G_,
                                    base_contact_Jac);

            Eigen::VectorQd torque_unbound; torque_unbound.setZero();
            bool qp_status = true;
            qddot_qp.setZero(); contact_wrench_qp.setZero(contact_dim);
            qp_status = dyn_wbc_.computeDynamicWBC(qddot_qp, contact_wrench_qp);
            // torque_unbound =  qddot_des.tail(MODEL_DOF);
            torque_unbound = (M_ * qddot_qp + G_ - base_contact_Jac.transpose() * contact_wrench_qp).tail(MODEL_DOF); 

            //--- Torque initialization
            static int tick_torque_desired_init = 0;
            if(is_torque_desired_init == true)
            {
                for (int i = 0; i < MODEL_DOF; i++) {
                    torque_unbound(i) = DyrosMath::cubic(tick_torque_desired_init, 0, 2000, torque_init(i), torque_unbound(i), 0.0, 0.0);
                }

                tick_torque_desired_init++;

                if(tick_torque_desired_init >= 2000) {
                    is_torque_desired_init = false;
                    std::cout << "##### INFO: INITIAL TORQUE SMOOTHING COMPLETE #####" << std::endl;
                }
            }

            //--- Torque saturation
            Eigen::VectorQd torque_bound;   torque_bound.setZero();
            for (int i = 0; i < MODEL_DOF; i++) {
                torque_bound(i) = DyrosMath::minmax_cut(torque_unbound(i), -rd_.torque_limit(i), rd_.torque_limit(i));
            }

            //--- Torque transition when contact state changes
            static int tick_transition = 0;
            if(is_left_contact_transition == true || is_right_contact_transition == true)
            {
                is_torque_transition = true;
                tick_transition = 0;
                torque_transition = torque_bound;
            }

            if(is_torque_transition == true)
            {
                for (int i = 0; i < MODEL_DOF; i++) {
                    torque_bound(i) = DyrosMath::cubic(tick_transition, 0, 200, torque_transition(i), torque_bound(i), 0.0, 0.0);
                }

                tick_transition++;

                if(tick_transition >= 200) {
                    is_torque_transition = false;
                }
            }

            //--- safety
            if(qp_status == true)
            {
                rd_.torque_desired = torque_bound;
            }
            else if (qp_status == false)
            {
                ROS_ERROR("QP feasibility violated. Emergency stop activated!");

                rd_.q_desired = rd_.q_;
                
                is_mode_7_working = false;
            }
        }
    }
    else
    {
        rd_.torque_desired = (Kp_diag * (rd_.q_desired - rd_.q_)) - (Kd_diag * rd_.q_dot_);
    }

    dataCC5 << where_am_i << std::endl;
    dataCC6 << rd_.torque_desired.transpose() << std::endl;
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
    Kp.setZero(MODEL_DOF); Kp_diag.setZero(MODEL_DOF, MODEL_DOF);         
    Kd.setZero(MODEL_DOF); Kd_diag.setZero(MODEL_DOF, MODEL_DOF);
    Ki.setZero(MODEL_DOF); Ki_diag.setZero(MODEL_DOF, MODEL_DOF);
    Kp_virtual.setZero(MODEL_DOF_VIRTUAL); Kp_virtual_diag.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);         
    Kd_virtual.setZero(MODEL_DOF_VIRTUAL); Kd_virtual_diag.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    Ki_virtual.setZero(MODEL_DOF_VIRTUAL); Ki_virtual_diag.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);

    std::vector<double> kp_vec, kd_vec, ki_vec;
    std::vector<double> pos_low_deg, pos_high_deg;
    std::vector<double> vel_low, vel_high;

    nh_cc_.getParam("/tocabi_controller/joint_gains/Kp", kp_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Kd", kd_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Ki", ki_vec);
    nh_cc_.getParam("/tocabi_controller/joint_limits/pos_low_deg", pos_low_deg);
    nh_cc_.getParam("/tocabi_controller/joint_limits/pos_high_deg", pos_high_deg);
    nh_cc_.getParam("/tocabi_controller/joint_limits/vel_low", vel_low);
    nh_cc_.getParam("/tocabi_controller/joint_limits/vel_high", vel_high);

    // Check Vector Dimension
    if (kp_vec.size() != MODEL_DOF_VIRTUAL)
        ROS_ERROR("Kp vector size mismatch: got %lu, expected %d", kp_vec.size(), MODEL_DOF_VIRTUAL);
    assert(kp_vec.size() == MODEL_DOF_VIRTUAL);

    if (kd_vec.size() != MODEL_DOF_VIRTUAL)
        ROS_ERROR("Kd vector size mismatch: got %lu, expected %d", kd_vec.size(), MODEL_DOF_VIRTUAL);
    assert(kd_vec.size() == MODEL_DOF_VIRTUAL);

    if (ki_vec.size() != MODEL_DOF_VIRTUAL)
        ROS_ERROR("Ki vector size mismatch: got %lu, expected %d", ki_vec.size(), MODEL_DOF_VIRTUAL);
    assert(ki_vec.size() == MODEL_DOF_VIRTUAL);

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
    for (int i = 0; i < MODEL_DOF_VIRTUAL; ++i)
    {
        Kp_virtual(i) = kp_vec[i];
        Kd_virtual(i) = kd_vec[i];
        Ki_virtual(i) = ki_vec[i];

        if(i >= 6)
        {
            Kp(i - 6) = kp_vec[i];
            Kd(i - 6) = kd_vec[i];
            Ki(i - 6) = ki_vec[i];
        }
    }

    Kp_virtual_diag = Kp_virtual.asDiagonal();
    Kd_virtual_diag = Kd_virtual.asDiagonal();
    Ki_virtual_diag = Ki_virtual.asDiagonal();
    Kp_diag = Kp.asDiagonal();
    Kd_diag = Kd.asDiagonal();
    Ki_diag = Ki.asDiagonal();

    // Position Limits (convert deg to rad)
    for (int i = 0; i < MODEL_DOF; ++i)
    {
        joint_pos_limit_l_(i) = pos_low_deg[i] * DEG2RAD;
        joint_pos_limit_h_(i) = pos_high_deg[i] * DEG2RAD;
    }

    //--- Velocity Limits
    for (int i = 0; i < MODEL_DOF; ++i)
    {
        joint_vel_limit_l_(i) = vel_low[i];
        joint_vel_limit_h_(i) = vel_high[i];
    }

    //--- Task Gain
    task_pos_Kp[base_link_name]  = 1.0 * Eigen::Vector3d::Ones();
    task_pos_Kp[chest_link_name] = 1.0 * Eigen::Vector3d::Ones();
    task_pos_Kp[head_link_name]  = 1.0 * Eigen::Vector3d::Ones();
    task_pos_Kp[lfoot_link_name] = 1.0 * Eigen::Vector3d::Ones();
    task_pos_Kp[rfoot_link_name] = 1.0 * Eigen::Vector3d::Ones();
    task_pos_Kp[lhand_link_name] = 1.0 * Eigen::Vector3d::Ones();
    task_pos_Kp[rhand_link_name] = 1.0 * Eigen::Vector3d::Ones();
    task_pos_Kp[com_name]        = 0.3 * Eigen::Vector3d::Ones();

    task_ori_Kp[base_link_name]  = 1.0 * Eigen::Vector3d::Ones();
    task_ori_Kp[chest_link_name] = 1.0 * Eigen::Vector3d::Ones();
    task_ori_Kp[head_link_name]  = 1.0 * Eigen::Vector3d::Ones();
    task_ori_Kp[lfoot_link_name] = 1.0 * Eigen::Vector3d::Ones();
    task_ori_Kp[rfoot_link_name] = 1.0 * Eigen::Vector3d::Ones();
    task_ori_Kp[lhand_link_name] = 5.0 * Eigen::Vector3d::Ones();
    task_ori_Kp[rhand_link_name] = 5.0 * Eigen::Vector3d::Ones();
    task_ori_Kp[com_name]        = 1.0 * Eigen::Vector3d::Ones();
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

    rd_.q_desired = DyrosMath::cubicVector<MODEL_DOF>(initial_tick, 0, 2.0 * hz_, q_init_, q_init_des, Eigen::VectorQd::Zero(), Eigen::VectorQd::Zero()); 

    initial_tick++;
}

void CustomController::stateManager()
{
    //--- Contact Change Trigger
    if(is_left_contact_transition == true)
    {
        contact_mode_ = ContactIndicator::LeftSingleSupport;
        WBC::SetContact(rd_, true, false);

        is_left_contact_transition = false;
    }
    else if(is_right_contact_transition == true)
    {
        contact_mode_ = ContactIndicator::RightSingleSupport;
        WBC::SetContact(rd_, false, true);

        is_right_contact_transition = false;
    }

    //--- Robot States
    Eigen::Vector3d base_pos = rd_.link_[link_index_map[base_link_name]].xpos; 
    Eigen::Matrix3d base_rot = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_.link_[link_index_map[base_link_name]].rotm)(2)); 
    for (const auto& [name, idx] : link_index_map)
    {
        //--- Global frame
        Jac_v[name] = rd_.link_[idx].Jac().topRows(3);
        Jac_w[name] = rd_.link_[idx].Jac().bottomRows(3);
        ee_pos[name] = rd_.link_[idx].xpos;
        ee_rot[name] = rd_.link_[idx].rotm;
        ee_v[name] = rd_.link_[idx].v;
        ee_w[name] = rd_.link_[idx].w;

        //--- Base frame
        base_Jac_v_prev[name] = base_Jac_v[name];
        base_Jac_w_prev[name] = base_Jac_w[name];
       
        base_Jac_v[name]  = base_rot.transpose() * Jac_v[name];
        base_Jac_w[name]  = base_rot.transpose() * Jac_w[name];
        base_Jac[name].topRows(3)    = base_Jac_v[name];
        base_Jac[name].bottomRows(3) = base_Jac_w[name];

        static bool is_jaco_dot_init = true;
        if (is_jaco_dot_init == true)
        {
            base_Jac_v_prev[name] = base_Jac_v[name];
            base_Jac_w_prev[name] = base_Jac_w[name];

            is_jaco_dot_init = false;
        }

        base_Jac_v_dot[name]  = (base_Jac_v[name] - base_Jac_v_prev[name]) * hz_;
        base_Jac_w_dot[name]  = (base_Jac_w[name] - base_Jac_w_prev[name]) * hz_;
        base_Jac_dot[name].topRows(3)    = base_Jac_v_dot[name];
        base_Jac_dot[name].bottomRows(3) = base_Jac_w_dot[name];

        base_ee_pos[name] = base_rot.transpose() * (ee_pos[name] - base_pos);
        base_ee_rot[name] = base_rot.transpose() *  ee_rot[name];                             
        base_ee_v[name]   = base_rot.transpose() *  ee_v[name];                               
        base_ee_w[name]   = base_rot.transpose() *  ee_w[name];
    }

    for (const auto& [name, idx] : link_index_map)
    {
        //--- Support frame
        if(contact_mode_ == ContactIndicator::DoubleSupport)
        {
            support_ee_pos[name] = base_ee_rot[lfoot_link_name] * (base_ee_pos[name] - base_ee_pos[lfoot_link_name]);
            support_ee_rot[name] = base_ee_rot[lfoot_link_name] * base_ee_rot[name];
            support_ee_v[name]   = base_ee_rot[lfoot_link_name] * base_ee_v[name];
            support_ee_w[name]   = base_ee_rot[lfoot_link_name] * base_ee_w[name]; 
        }
        else if (contact_mode_ == ContactIndicator::LeftSingleSupport)
        {
            support_ee_pos[name] = base_ee_rot[lfoot_link_name] * (base_ee_pos[name] - base_ee_pos[lfoot_link_name]);
            support_ee_rot[name] = base_ee_rot[lfoot_link_name] * base_ee_rot[name];
            support_ee_v[name]   = base_ee_rot[lfoot_link_name] * base_ee_v[name];
            support_ee_w[name]   = base_ee_rot[lfoot_link_name] * base_ee_w[name]; 
        }
        else if (contact_mode_ == ContactIndicator::RightSingleSupport)
        {
            support_ee_pos[name] = base_ee_rot[rfoot_link_name] * (base_ee_pos[name] - base_ee_pos[rfoot_link_name]);
            support_ee_rot[name] = base_ee_rot[rfoot_link_name] * base_ee_rot[name];
            support_ee_v[name]   = base_ee_rot[rfoot_link_name] * base_ee_v[name];
            support_ee_w[name]   = base_ee_rot[rfoot_link_name] * base_ee_w[name]; 
        }
        else
        {
            ROS_ERROR("Contact Indicator are assigned with something wrong value.");
            assert(contact_mode_ == ContactIndicator::DoubleSupport || contact_mode_ == ContactIndicator::LeftSingleSupport || contact_mode_ == ContactIndicator::RightSingleSupport);
        }
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
    M_inv_ = M_.llt().solve(MatrixXd::Identity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL));
    
    // RigidBodyDynamics::NonlinearEffects(model_, base_q_virtual_, Eigen::VectorXd::Zero(MODEL_DOF_QVIRTUAL), G_temp_);
    // G_ = G_temp_;

    //--- Joint State w.r.t. base frame
    q_.segment(0,3) = base_ee_pos[base_link_name];
    q_.segment(3,3) = DyrosMath::rot2Euler(base_ee_rot[base_link_name]);
    q_.segment(6,MODEL_DOF) = rd_.q_;
    
    qdot_.segment(0,3) = base_ee_v[base_link_name];
    qdot_.segment(3,3) = base_ee_w[base_link_name];
    qdot_.segment(6,MODEL_DOF) = rd_.q_dot_;
    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
    {
        qdot_LPF(i) = DyrosMath::lpf(qdot_(i), qdot_LPF(i), 2000.0, 10.0);
    }
    
    RigidBodyDynamics::NonlinearEffects(model_, q_, qdot_, G_temp_);
    G_ = G_temp_;

    S_T.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF); S_T.bottomRows(MODEL_DOF).setIdentity();
    S.setZero(MODEL_DOF, MODEL_DOF_VIRTUAL);   S = S_T.transpose();

    // --- Reachability (# TODO: Put into kin_wbc.cpp)
    struct ReachPair {
        std::string name_A; std::string name_B; double max_dist{0.0};
    };

    const std::vector<ReachPair> reach_pairs = {
        {lshoulder_link_name, lhand_link_name, 0.5},
        {rshoulder_link_name, rhand_link_name, 0.5},
    };

    const int m = static_cast<int>(reach_pairs.size());

    std::vector<Eigen::MatrixXd> J_reachability; J_reachability.reserve(m);                
    std::vector<double> h_reachability; h_reachability.reserve(m);

    for (int i = 0; i < m; ++i) {
        const auto& pr = reach_pairs[i];
        const int idA = link_index_map.at(pr.name_A);
        const int idB = link_index_map.at(pr.name_B);

        Eigen::MatrixXd J_; 
        double dist_bwt_linkA_linkB = getSignedDistanceFunction(rd_.link_[idA], rd_.link_[idB], J_);

        J_reachability.push_back(-J_);
        h_reachability.push_back(-dist_bwt_linkA_linkB + pr.max_dist);
    }

    kin_wbc_.getReachabilityConstraints(J_reachability, h_reachability);
}

void CustomController::contactStateManager()
{
    //--- Contact Dimension
    if(contact_mode_ == ContactIndicator::DoubleSupport)
    {
        contact_dim = 12;
    }
    else if(contact_mode_ == ContactIndicator::LeftSingleSupport || contact_mode_ == ContactIndicator::RightSingleSupport)
    {
        contact_dim = 6;
    }

    base_contact_Jac.setZero(contact_dim, MODEL_DOF_VIRTUAL);
    base_contact_Jac_dot.setZero(contact_dim, MODEL_DOF_VIRTUAL);

    if(contact_mode_ == ContactIndicator::DoubleSupport)
    {
        base_contact_Jac.block(0, 0, 6, MODEL_DOF_VIRTUAL) = base_Jac[lfoot_link_name]; 
        base_contact_Jac.block(6, 0, 6, MODEL_DOF_VIRTUAL) = base_Jac[rfoot_link_name]; 

        base_contact_Jac_dot.block(0, 0, 6, MODEL_DOF_VIRTUAL) = base_Jac_dot[lfoot_link_name]; 
        base_contact_Jac_dot.block(6, 0, 6, MODEL_DOF_VIRTUAL) = base_Jac_dot[rfoot_link_name];

        support_zmp_ref = (init_support_ee_pos.at(lfoot_link_name) + init_support_ee_pos.at(rfoot_link_name)) / 2.0;
    }
    else if(contact_mode_ == ContactIndicator::LeftSingleSupport)
    {
        base_contact_Jac = base_Jac[lfoot_link_name]; 

        base_contact_Jac_dot = base_Jac_dot[lfoot_link_name]; 

        support_zmp_ref = init_support_ee_pos.at(lfoot_link_name);
    }
    else if(contact_mode_ == ContactIndicator::RightSingleSupport)
    {
        base_contact_Jac = base_Jac[rfoot_link_name]; 

        base_contact_Jac_dot = base_Jac_dot[rfoot_link_name]; 

        support_zmp_ref = init_support_ee_pos.at(rfoot_link_name);
    }
    else
    {
        ROS_ERROR("Contact Indicator are assigned with something wrong value.");
        assert(contact_mode_ == ContactIndicator::DoubleSupport || contact_mode_ == ContactIndicator::LeftSingleSupport || contact_mode_ == ContactIndicator::RightSingleSupport);
    }

    base_contact_lambda.setZero(contact_dim, contact_dim);
    base_contact_lambda = (base_contact_Jac * M_inv_ * base_contact_Jac.transpose()).llt().solve(MatrixXd::Identity(contact_dim, contact_dim));

    base_contact_Jac_inv_T.setZero(contact_dim, MODEL_DOF_VIRTUAL);
    base_contact_Jac_inv_T = base_contact_lambda * base_contact_Jac * M_inv_;

    base_contact_N.setZero();
    base_contact_N = MatrixXd::Identity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) - base_contact_Jac.transpose() * base_contact_Jac_inv_T;
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

    init_support_ee_pos = support_ee_pos;
    init_support_ee_rot = support_ee_rot;
    init_support_ee_v = support_ee_v;
    init_support_ee_w = support_ee_w;

    rd_.q_desired = q_init_des;

    for (const auto& [name, idx] : link_index_map)
    {
        x_desired[name]   = init_support_ee_pos[name];
        dx_desired[name]  = Eigen::Vector3d::Zero();
        ddx_desired[name] = Eigen::Vector3d::Zero();

        R_desired[name]   = init_support_ee_rot[name];
        w_desired[name]   = Eigen::Vector3d::Zero();
        dw_desired[name]  = Eigen::Vector3d::Zero();

        support_x_desired[name]   = init_support_ee_pos[name];
        support_dx_desired[name]  = Eigen::Vector3d::Zero();
        support_ddx_desired[name] = Eigen::Vector3d::Zero();
        support_R_desired[name]   = init_support_ee_rot[name];
        support_w_desired[name]   = Eigen::Vector3d::Zero();
        support_dw_desired[name]  = Eigen::Vector3d::Zero();
    }

    q_des.segment(6, MODEL_DOF)= q_init_des;
    dq_des.setZero(); 
    qdot_des.setZero();
    qddot_des.setZero(); 

    torque_init = (Kp_diag * (q_init_des - rd_.q_)) - (Kd_diag * rd_.q_dot_);
}

void CustomController::runTestMotion(const double& traj_time, const double& pelv_dist, const double& hand_dist, const double& foot_height, const double& swing_duration)
{
    switch (motion_mode_)
    {
        case TestMotionType::PelvHand:
            movePelvHandPose(traj_time, pelv_dist, hand_dist);
            break;
        case TestMotionType::PelvHandJoy:
            movePelvHandPoseJoy(target_vel_x_, target_vel_y_, target_vel_yaw_, traj_time, hand_dist);
            break;
        case TestMotionType::Taichi:
            moveTaichiMotion(traj_time, pelv_dist, hand_dist, foot_height);
            break;
        case TestMotionType::None:
        default:
            break;
    }
}

void CustomController::movePelvHandPose(double traj_time, double pelv_dist, double hand_dist)
{
    task_hierarchy = {
        {{com_name, TaskType::Position}},
        {{base_link_name, TaskType::Orientation}},
        {{chest_link_name, TaskType::Orientation}},
        {{head_link_name, TaskType::Orientation}},
        {{lhand_link_name, TaskType::Position}, {lhand_link_name, TaskType::Orientation}},
        {{rhand_link_name, TaskType::Position}, {rhand_link_name, TaskType::Orientation}}};

    std::set<std::string> task_names;
    for (const auto& task_group : task_hierarchy)
    {
        for (const auto& [name, type] : task_group)
        {
            if (task_names.count(name))
                continue;
            task_names.insert(name);
        }
    }
    //--- Initialization
    static int tick = 0;
    for (const auto& name : task_names)
    {
        x_desired.at(name) = init_support_ee_pos.at(name);
        dx_desired.at(name).setZero();
        ddx_desired.at(name).setZero();

        R_desired.at(name) = init_support_ee_rot.at(name);
        w_desired.at(name).setZero();
        dw_desired.at(name).setZero();
    }

    //--- Pelvis Test
    x_desired[com_name](1) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                   init_support_ee_pos[com_name](1), 
                                                   init_support_ee_pos[com_name](1) + pelv_dist, 
                                                   0.0, 0.0);

    dx_desired[com_name](1) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                       init_support_ee_pos[com_name](1), 
                                                       init_support_ee_pos[com_name](1) + pelv_dist, 
                                                       0.0, 0.0);

    ddx_desired[com_name](1) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                         init_support_ee_pos[com_name](1), 
                                                         init_support_ee_pos[com_name](1) + pelv_dist, 
                                                         0.0, 0.0);
    // static int tick_pelv = 0;
    // double T = traj_time; // period
    // double wn = (2.0 * M_PI) / T;
    // if(is_torque_desired_init == false)
    // {
    //     x_desired[com_name](1)   = init_support_ee_pos[com_name](1) + pelv_dist * sin(wn * tick_pelv / hz_);                                          
    //     dx_desired[com_name](1)  = wn * pelv_dist * cos(wn * tick_pelv / hz_);                                          
    //     ddx_desired[com_name](1) = (-1.0) * wn * wn * pelv_dist * sin(wn * tick_pelv / hz_);                                          
    //     tick_pelv++;
    // }

    //--- Hand Test
    for (int idx = 2; idx < 3; idx++)
    {
        x_desired[lhand_link_name](idx) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                        init_support_ee_pos[lhand_link_name](idx), 
                                                        init_support_ee_pos[lhand_link_name](idx) + hand_dist, 
                                                        0.0, 0.0);

        dx_desired[lhand_link_name](idx) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                            init_support_ee_pos[lhand_link_name](idx), 
                                                            init_support_ee_pos[lhand_link_name](idx) + hand_dist, 
                                                            0.0, 0.0);

        ddx_desired[lhand_link_name](idx) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                            init_support_ee_pos[lhand_link_name](idx), 
                                                            init_support_ee_pos[lhand_link_name](idx) + hand_dist, 
                                                            0.0, 0.0);

        x_desired[rhand_link_name](idx) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                        init_support_ee_pos[rhand_link_name](idx), 
                                                        init_support_ee_pos[rhand_link_name](idx) - hand_dist, 
                                                        0.0, 0.0);

        dx_desired[rhand_link_name](idx) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                            init_support_ee_pos[rhand_link_name](idx), 
                                                            init_support_ee_pos[rhand_link_name](idx) - hand_dist, 
                                                            0.0, 0.0);

        ddx_desired[rhand_link_name](idx) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                            init_support_ee_pos[rhand_link_name](idx), 
                                                            init_support_ee_pos[rhand_link_name](idx) - hand_dist, 
                                                            0.0, 0.0);
    }

    //--- Data Logging
    dataCC1 << x_desired[com_name].transpose()  << " " << support_ee_pos[com_name].transpose() << std::endl;
    dataCC2 << support_dcm_des.transpose() << " " << support_dcm_mea.transpose() << std::endl;
    dataCC3 << support_zmp_ref.transpose() << " " << support_zmp_des.transpose() << std::endl;
    dataCC4 << init_support_ee_pos[lfoot_link_name].transpose() << " " << init_support_ee_pos[rfoot_link_name].transpose() << std::endl;
    
    //--- Map Desired to base frame
    for (const auto& name : task_names)
    {
        if (contact_mode_ == ContactIndicator::DoubleSupport)
        {
            x_desired.at(name)   = init_support_ee_rot[lfoot_link_name].transpose() * (x_desired.at(name) - support_ee_pos[base_link_name]);
            dx_desired.at(name)  = init_support_ee_rot[lfoot_link_name].transpose() * dx_desired.at(name);
            ddx_desired.at(name) = init_support_ee_rot[lfoot_link_name].transpose() * ddx_desired.at(name);

            R_desired.at(name)   = init_support_ee_rot[lfoot_link_name].transpose() * R_desired.at(name);
            w_desired.at(name)   = init_support_ee_rot[lfoot_link_name].transpose() * w_desired.at(name);
            dw_desired.at(name)  = init_support_ee_rot[lfoot_link_name].transpose() * dw_desired.at(name);
        }
    }

    //--- Increment Tick
    tick++;
}

void CustomController::movePelvHandPoseJoy(const double& vx, const double& vy, const double& wz, const double& traj_time, const double& hand_dist)
{
    task_hierarchy= {
            { {base_link_name,  TaskType::Position}, {base_link_name, TaskType::Orientation} },
            { {chest_link_name, TaskType::Orientation} },
            { {head_link_name,  TaskType::Orientation} },
            { {lhand_link_name, TaskType::Position}, {lhand_link_name, TaskType::Orientation} },
            { {rhand_link_name, TaskType::Position}, {rhand_link_name, TaskType::Orientation} }
    };

    std::set<std::string> task_names;
    for (const auto& task_group : task_hierarchy)
    {
        for (const auto& [name, type] : task_group)
        {
            if (task_names.count(name))
                continue;
            task_names.insert(name);
        }
    }
    
    //--- Initialization
    static int tick = 0;

    //--- Pelvis Test
    support_x_desired[base_link_name](0) += vx / hz_;
    support_x_desired[base_link_name](1) += vy / hz_;

    support_dx_desired[base_link_name](0) = vx;
    support_dx_desired[base_link_name](1) = vy;

    //--- Pelvis Orientation
    support_w_desired[base_link_name](2) = wz;

    Eigen::Vector3d eulerDot_desired = AngvelToEulerRates(w_desired[base_link_name], DyrosMath::rot2Euler(base_ee_rot[base_link_name]));
    Eigen::Vector3d euler_desired = eulerDot_desired / hz_;
    R_desired[base_link_name] = DyrosMath::Euler2rot(euler_desired(0), euler_desired(1), euler_desired(2));

    //--- Hand Test
    for (int idx = 1; idx < 3; idx++)
    {
        support_x_desired[lhand_link_name](idx) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                           init_support_ee_pos[lhand_link_name](idx), 
                                                           init_support_ee_pos[lhand_link_name](idx) + hand_dist, 
                                                           0.0, 0.0);

        support_dx_desired[lhand_link_name](idx) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                               init_support_ee_pos[lhand_link_name](idx), 
                                                               init_support_ee_pos[lhand_link_name](idx) + hand_dist, 
                                                               0.0, 0.0);

        support_ddx_desired[lhand_link_name](idx) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                                 init_support_ee_pos[lhand_link_name](idx), 
                                                                 init_support_ee_pos[lhand_link_name](idx) + hand_dist, 
                                                                 0.0, 0.0);

        support_x_desired[rhand_link_name](idx) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                           init_support_ee_pos[rhand_link_name](idx), 
                                                           init_support_ee_pos[rhand_link_name](idx) - hand_dist, 
                                                           0.0, 0.0);

        support_dx_desired[rhand_link_name](idx) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                               init_support_ee_pos[rhand_link_name](idx), 
                                                               init_support_ee_pos[rhand_link_name](idx) - hand_dist, 
                                                               0.0, 0.0);

        support_ddx_desired[rhand_link_name](idx) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                                 init_support_ee_pos[rhand_link_name](idx), 
                                                                 init_support_ee_pos[rhand_link_name](idx) - hand_dist, 
                                                                 0.0, 0.0);
    }

    //--- Data Logging
    dataCC1 << support_x_desired[base_link_name].transpose()  << " " << support_ee_pos[base_link_name].transpose() << std::endl;
    
    //--- Map Desired to base frame
    for (const auto& name : task_names)
    {
        if (contact_mode_ == ContactIndicator::DoubleSupport || contact_mode_ == ContactIndicator::LeftSingleSupport)
        {
            x_desired.at(name)   = init_support_ee_rot[lfoot_link_name].transpose() * (support_x_desired.at(name) - support_ee_pos[base_link_name]);
            dx_desired.at(name)  = init_support_ee_rot[lfoot_link_name].transpose() *  support_dx_desired.at(name);
            ddx_desired.at(name) = init_support_ee_rot[lfoot_link_name].transpose() *  support_ddx_desired.at(name);

            R_desired.at(name)   = init_support_ee_rot[lfoot_link_name].transpose() * support_R_desired.at(name);
            w_desired.at(name)   = init_support_ee_rot[lfoot_link_name].transpose() * support_w_desired.at(name);
            dw_desired.at(name)  = init_support_ee_rot[lfoot_link_name].transpose() * support_dw_desired.at(name);
        }
    }

    //--- Increment Tick
    tick++;
}

void CustomController::moveTaichiMotion(const double& traj_time, const double& pelv_dist, const double& hand_dist, const double& foot_height)
{
    static int tick = 0;

    task_hierarchy= {
            { {base_link_name,  TaskType::Position}, {base_link_name, TaskType::Orientation} },
            { {chest_link_name, TaskType::Orientation} },
            { {head_link_name,  TaskType::Orientation} },
            { {lhand_link_name, TaskType::Position}, {lhand_link_name, TaskType::Orientation} },
            { {rhand_link_name, TaskType::Position}, {rhand_link_name, TaskType::Orientation} }
    };

    if ( tick >= traj_time * hz_)
    {
        task_hierarchy= {
                { {base_link_name,  TaskType::Position}, {base_link_name, TaskType::Orientation} },
                { {rfoot_link_name, TaskType::Position}, {rfoot_link_name, TaskType::Orientation} },
                { {chest_link_name, TaskType::Orientation} },
                { {head_link_name,  TaskType::Orientation} },
                { {lhand_link_name, TaskType::Position}, {lhand_link_name, TaskType::Orientation} },
                { {rhand_link_name, TaskType::Position}, {rhand_link_name, TaskType::Orientation} }
        };
    }

    std::set<std::string> task_names;
    for (const auto& task_group : task_hierarchy)
    {
        for (const auto& [name, type] : task_group)
        {
            if (task_names.count(name))
                continue;
            task_names.insert(name);
        }
    }

    //--- Initialization
    for (const auto& name : task_names)
    {
        x_desired.at(name) = init_support_ee_pos.at(name);
        dx_desired.at(name).setZero();
        ddx_desired.at(name).setZero();

        R_desired.at(name) = init_support_ee_rot.at(name);
        w_desired.at(name).setZero();
        dw_desired.at(name).setZero();
    }

    //--- Pelvis Test
    Eigen::Vector3d pelv_traj; pelv_traj.setZero();
    pelv_traj = DyrosMath::QuinticSpline(tick, 0, traj_time * hz_, 
                                            init_support_ee_pos[base_link_name](1), 0.0, 0.0, 
                                            init_support_ee_pos[base_link_name](1) + pelv_dist, 0.0, 0.0);

    x_desired[base_link_name](1)   = pelv_traj(0);
    dx_desired[base_link_name](1)  = pelv_traj(1);
    ddx_desired[base_link_name](1) = pelv_traj(2);

    //--- Hand Test
    for (int idx = 1; idx < 3; idx++)
    {
        x_desired[lhand_link_name](idx) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                        init_support_ee_pos[lhand_link_name](idx), 
                                                        init_support_ee_pos[lhand_link_name](idx) + hand_dist, 
                                                        0.0, 0.0);

        dx_desired[lhand_link_name](idx) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                            init_support_ee_pos[lhand_link_name](idx), 
                                                            init_support_ee_pos[lhand_link_name](idx) + hand_dist, 
                                                            0.0, 0.0);

        ddx_desired[lhand_link_name](idx) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                            init_support_ee_pos[lhand_link_name](idx), 
                                                            init_support_ee_pos[lhand_link_name](idx) + hand_dist, 
                                                            0.0, 0.0);

        x_desired[rhand_link_name](idx) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                        init_support_ee_pos[rhand_link_name](idx), 
                                                        init_support_ee_pos[rhand_link_name](idx) - hand_dist, 
                                                        0.0, 0.0);

        dx_desired[rhand_link_name](idx) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                            init_support_ee_pos[rhand_link_name](idx), 
                                                            init_support_ee_pos[rhand_link_name](idx) - hand_dist, 
                                                            0.0, 0.0);

        ddx_desired[rhand_link_name](idx) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                            init_support_ee_pos[rhand_link_name](idx), 
                                                            init_support_ee_pos[rhand_link_name](idx) - hand_dist, 
                                                            0.0, 0.0);
    }

    // //--- Swing Foot Test
    x_desired[rfoot_link_name] = init_support_ee_pos[rfoot_link_name];
    R_desired[rfoot_link_name] = init_support_ee_rot[rfoot_link_name];

    x_desired[rfoot_link_name](2) = DyrosMath::cubic(tick, traj_time * hz_, 2.0 * traj_time * hz_, 
                                                        init_support_ee_pos[rfoot_link_name](2), 
                                                        init_support_ee_pos[rfoot_link_name](2) + foot_height, 
                                                        0.0, 0.0);

    dx_desired[rfoot_link_name](2) = DyrosMath::cubicDot(tick, traj_time * hz_, 2.0 * traj_time * hz_, 
                                                            init_support_ee_pos[rfoot_link_name](2), 
                                                            init_support_ee_pos[rfoot_link_name](2) + foot_height, 
                                                            0.0, 0.0);

    ddx_desired[rfoot_link_name](2) = DyrosMath::cubicDdot(tick, traj_time * hz_, 2.0 * traj_time * hz_, 
                                                            init_support_ee_pos[rfoot_link_name](2), 
                                                            init_support_ee_pos[rfoot_link_name](2) + foot_height, 
                                                            0.0, 0.0);
                                                            
    dataCC1 << x_desired[base_link_name].transpose()  << " " << support_ee_pos[base_link_name].transpose() << std::endl;

    //--- Map Desired to base frame
    for (const auto& name : task_names)
    {
        if (contact_mode_ == ContactIndicator::DoubleSupport || contact_mode_ == ContactIndicator::LeftSingleSupport)
        {
            x_desired.at(name)   = init_support_ee_rot[lfoot_link_name].transpose() * (x_desired.at(name) - support_ee_pos[base_link_name]);
            dx_desired.at(name)  = init_support_ee_rot[lfoot_link_name].transpose() * dx_desired.at(name);
            ddx_desired.at(name) = init_support_ee_rot[lfoot_link_name].transpose() * ddx_desired.at(name);

            R_desired.at(name)   = init_support_ee_rot[lfoot_link_name].transpose() * R_desired.at(name);
            w_desired.at(name)   = init_support_ee_rot[lfoot_link_name].transpose() * w_desired.at(name);
            dw_desired.at(name)  = init_support_ee_rot[lfoot_link_name].transpose() * dw_desired.at(name);
        }
    }

    //--- Increment Tick
    tick++;

    if ( tick == traj_time * hz_ - 1)    {
        is_left_contact_transition = true;  // Update state transition next tick
    }
}

//--- Signed Distance Function
double CustomController::getSignedDistanceFunction(LinkData &linkA_, LinkData &linkB_, Eigen::MatrixXd &J_AB)
{   
    // Initialization
    double sd_AB = 0.0;

    Eigen::Vector3d posA_transform_current_from_global_; posA_transform_current_from_global_.setZero();
    Eigen::Vector3d posB_transform_current_from_global_; posB_transform_current_from_global_.setZero();

    // Base Coordinate
    Eigen::Vector3d base_pos = rd_.link_[link_index_map[base_link_name]].xpos; 
    Eigen::Matrix3d base_rot = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_.link_[link_index_map[base_link_name]].rotm)(2)); 

    posA_transform_current_from_global_ = base_rot.transpose() * (linkA_.xpos - base_pos);
    posB_transform_current_from_global_ = base_rot.transpose() * (linkB_.xpos - base_pos);

    sd_AB = (posA_transform_current_from_global_ - posB_transform_current_from_global_).norm(); 

    Eigen::Vector3d normal_vector_btw_AB; normal_vector_btw_AB.setZero();
    normal_vector_btw_AB = (posA_transform_current_from_global_ - posB_transform_current_from_global_) / (posA_transform_current_from_global_ - posB_transform_current_from_global_).norm();

    J_AB.setZero(1, MODEL_DOF_VIRTUAL);
    J_AB = normal_vector_btw_AB.transpose() * base_rot.transpose() * (linkA_.Jac().topRows(3) - linkB_.Jac().topRows(3));

    return sd_AB;
}


//--- Joy Utils
void CustomController::xBoxJoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double vel_threshold = 0.1;

    target_vel_x_   = DyrosMath::minmax_cut(joy->axes[1] * vel_threshold, -vel_threshold, vel_threshold);
    target_vel_y_   = DyrosMath::minmax_cut(joy->axes[0] * vel_threshold, -vel_threshold, vel_threshold);
    target_vel_yaw_ = DyrosMath::minmax_cut(joy->axes[3] * vel_threshold, -vel_threshold, vel_threshold);
}

