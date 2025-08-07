#include "cc.h"

using namespace TOCABI;

ofstream dataCC1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC1.txt");
ofstream dataCC2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC2.txt");
ofstream dataCC3("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC3.txt");
ofstream dataCC4("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC4.txt");
ofstream dataCC5("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC5.txt");
ofstream dataCC6("/home/kwan/catkin_ws/src/tocabi_cc/data/dataCC6.txt");

CustomController::CustomController(RobotData &rd) : rd_(rd), dyn_wbc_(MODEL_DOF_VIRTUAL)
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
    
    if (rd_.tc_.mode == 6)
    {   
        if (is_mode_6_init == true)
        {
            loadParams();

            q_init_ = rd_.q_;
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
        if(is_mode_7_working == true)
        {
            stateManager();

            if(is_mode_7_init == true)
            {
                saveInitialState();
                dyn_wbc_.setRobotSystemParameters(0.8,                 // Friction Coefficient
                                                0.3,                 // Foot size
                                                0.16,                // Foot width
                                                2000.0,              // Max vertical contact force
                                                0.0,                 // Min Vertical contact force                             
                                                rd_.torque_limit,    // Torque limit
                                                joint_pos_limit_l_,
                                                joint_pos_limit_h_,
                                                joint_vel_limit_l_,
                                                joint_vel_limit_h_);

                cout << "COMPUTESLOW MODE 7 IS NOW INITIALIZED" << endl;
                cout << "TIME: "<< rd_.control_time_ << endl; 

                is_mode_7_init = false;
            }

            contactStateManager();

            // motion_mode_ = TestMotionType::Taichi;
            // motion_mode_ = TestMotionType::PelvHand;
            motion_mode_ = TestMotionType::PelvHandJoy;
            runTestMotion(5.0, 0.1, 0.2, 0.2, 0.6);

            taskStateManager();

            //--- Whole-body Control
            for (const auto& task_group : task_hierarchy)
            {
                int m = 3 * task_group.size();
                for (const auto& [name, type] : task_group){ W_task[name] = 1.0 * Eigen::VectorXd::Ones(m);}
            }

            W_contact = 1.0 * Eigen::VectorXd::Ones(contact_dim);
            W_energy = 10.0 * Eigen::VectorQd::Ones();
            W_torque_prev = 0.0 * Eigen::VectorQd::Ones();

            dyn_wbc_.setWbcWeights(task_hierarchy, W_task, W_energy, W_contact, W_torque_prev);

            dyn_wbc_.computeTaskImpedance(task_hierarchy,
                                        task_Kp, task_Kv,
                                        x_desired, dx_desired, ddx_desired,
                                        R_desired, w_desired, dw_desired,
                                        base_ee_pos, base_ee_rot,
                                        base_ee_v, base_ee_w);

            dyn_wbc_.computeContactWrench(contact_mode_, rd_.link_[COM_id].mass * GRAVITY);

            dyn_wbc_.getRobotStates(task_hierarchy,
                                    q_,
                                    qdot_,
                                    M_, 
                                    M_inv_, 
                                    G_, 
                                    base_contact_Jac,
                                    base_contact_Jac_dot,
                                    base_contact_lambda,
                                    base_contact_Jac_inv_T,
                                    base_contact_N, 
                                    base_task_Jac_inv_T_S_T,
                                    rd_.torque_desired);

            //--- Compute QP-based Whole-body Controller 
            Eigen::VectorQd torque_unbound; torque_unbound.setZero();
            bool qp_status = true;
            qp_status = dyn_wbc_.computeDynamicWBC(task_hierarchy, torque_unbound);

            //--- Torque saturation
            Eigen::VectorQd torque_bound;   torque_bound.setZero();
            for (int i = 0; i < MODEL_DOF; i++) {
                torque_bound(i) = DyrosMath::minmax_cut(torque_unbound(i), -rd_.torque_limit(i), rd_.torque_limit(i));
            }

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

            dataCC6 << torque_bound.transpose() << std::endl;
        }
    }
    else
    {
        rd_.torque_desired = (Kp_diag * (rd_.q_desired - rd_.q_)) - (Kd_diag * rd_.q_dot_);
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
    Kp.setZero(MODEL_DOF); Kp_diag.setZero(MODEL_DOF, MODEL_DOF);         
    Kd.setZero(MODEL_DOF); Kd_diag.setZero(MODEL_DOF, MODEL_DOF);
    Kp_virtual.setZero(MODEL_DOF_VIRTUAL); Kp_virtual_diag.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);         
    Kd_virtual.setZero(MODEL_DOF_VIRTUAL); Kd_virtual_diag.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);

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
    if (kp_vec.size() != MODEL_DOF_VIRTUAL)
        ROS_ERROR("Kp vector size mismatch: got %lu, expected %d", kp_vec.size(), MODEL_DOF_VIRTUAL);
    assert(kp_vec.size() == MODEL_DOF_VIRTUAL);

    if (kd_vec.size() != MODEL_DOF_VIRTUAL)
        ROS_ERROR("Kd vector size mismatch: got %lu, expected %d", kd_vec.size(), MODEL_DOF_VIRTUAL);
    assert(kd_vec.size() == MODEL_DOF_VIRTUAL);

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

        if(i >= 6)
        {
            Kp(i - 6) = kp_vec[i];
            Kd(i - 6) = kd_vec[i];
        }
    }

    Kp_virtual_diag = Kp_virtual.asDiagonal();
    Kd_virtual_diag = Kd_virtual.asDiagonal();
    Kp_diag = Kp.asDiagonal();
    Kd_diag = Kd.asDiagonal();

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
    task_Kp[base_link_name]  = 40000.0 * Eigen::Vector3d::Ones();
    task_Kp[base_link_name](0) = 10000.0;
    task_Kp[base_link_name](1) = 10000.0;
    task_Kp[chest_link_name] = 40000.0 * Eigen::Vector3d::Ones();
    task_Kp[head_link_name]  = 40000.0 * Eigen::Vector3d::Ones();
    task_Kp[lfoot_link_name] = 40000.0 * Eigen::Vector3d::Ones();
    task_Kp[rfoot_link_name] = 40000.0 * Eigen::Vector3d::Ones();
    task_Kp[lhand_link_name] = 40000.0 * Eigen::Vector3d::Ones();
    task_Kp[rhand_link_name] = 40000.0 * Eigen::Vector3d::Ones();
    task_Kp[com_name]        = 40000.0 * Eigen::Vector3d::Ones();

    task_Kv[base_link_name]  = 400.0 * Eigen::Vector3d::Ones();
    task_Kv[base_link_name](0) = 200.0;
    task_Kv[base_link_name](1) = 200.0;
    task_Kv[chest_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kv[head_link_name]  = 400.0 * Eigen::Vector3d::Ones();
    task_Kv[lfoot_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kv[rfoot_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kv[lhand_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kv[rhand_link_name] = 400.0 * Eigen::Vector3d::Ones();
    task_Kv[com_name]        = 400.0 * Eigen::Vector3d::Ones();
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
    Eigen::Vector3d base_pos = rd_.link_[Pelvis].xpos; 
    Eigen::Matrix3d base_rot = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(2)); 
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
    
    RigidBodyDynamics::NonlinearEffects(model_, base_q_virtual_, qdot_, G_temp_);
    G_ = G_temp_;
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
    W_contact.setZero(contact_dim);

    if(contact_mode_ == ContactIndicator::DoubleSupport)
    {
        base_contact_Jac.block(0, 0, 6, MODEL_DOF_VIRTUAL) = base_Jac[lfoot_link_name]; 
        base_contact_Jac.block(6, 0, 6, MODEL_DOF_VIRTUAL) = base_Jac[rfoot_link_name]; 

        base_contact_Jac_dot.block(0, 0, 6, MODEL_DOF_VIRTUAL) = base_Jac_dot[lfoot_link_name]; 
        base_contact_Jac_dot.block(6, 0, 6, MODEL_DOF_VIRTUAL) = base_Jac_dot[rfoot_link_name];
    }
    else if(contact_mode_ == ContactIndicator::LeftSingleSupport)
    {
        base_contact_Jac = base_Jac[lfoot_link_name]; 

        base_contact_Jac_dot = base_Jac_dot[lfoot_link_name]; 
    }
    else if(contact_mode_ == ContactIndicator::RightSingleSupport)
    {
        base_contact_Jac = base_Jac[rfoot_link_name]; 

        base_contact_Jac_dot = base_Jac_dot[rfoot_link_name]; 
    }
    else
    {
        ROS_ERROR("Contact Indicator are assigned with something wrong value.");
        assert(contact_mode_ == ContactIndicator::DoubleSupport || contact_mode_ == ContactIndicator::LeftSingleSupport || contact_mode_ == ContactIndicator::RightSingleSupport);
    }

    //--- Contact Consistent Control Framework for Humanoid Robots
    base_contact_lambda.setZero(contact_dim, contact_dim);
    base_contact_lambda = (base_contact_Jac * M_inv_ * base_contact_Jac.transpose()).llt().solve(MatrixXd::Identity(contact_dim, contact_dim));

    base_contact_Jac_inv_T.setZero(contact_dim, MODEL_DOF_VIRTUAL);
    base_contact_Jac_inv_T = base_contact_lambda * base_contact_Jac * M_inv_;

    base_contact_N.setZero();
    base_contact_N = MatrixXd::Identity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) - base_contact_Jac.transpose() * base_contact_Jac_inv_T;
}

void CustomController::taskStateManager()
{
    for (const auto& task_group : task_hierarchy)
    {
        int task_dim = 0;
        Eigen::MatrixXd J_task;

        if (task_group.size() == 2)
        {
            task_dim = 6;
            const auto& task_info = task_group[0];  

            J_task.setZero(task_dim, MODEL_DOF_VIRTUAL);
            J_task = base_Jac[task_info.link_name]; 
        }
        else if (task_group.size() == 1)
        {
            task_dim = 3;
            const auto& task_info = task_group[0];  

            J_task.setZero(task_dim, MODEL_DOF_VIRTUAL);

            if (task_info.type == TaskType::Position)
            {
                J_task = base_Jac[task_info.link_name].topRows(3);
            }
            else if (task_info.type == TaskType::Orientation)
            {
                J_task = base_Jac[task_info.link_name].bottomRows(3);
            }
            else
            {
                ROS_ERROR("Unknown TaskType for link [%s]", task_info.link_name.c_str());
                continue;
            }
        }
        else
        {
            ROS_ERROR("Unexpected task_group size: %zu", task_group.size());
            continue;
        }

        std::string name = task_group[0].link_name; 
        W_task[name].setZero(task_dim);

        base_task_lambda[name].setZero(task_dim, task_dim);
        base_task_lambda[name] = (J_task * M_inv_ * base_contact_N * J_task.transpose()).llt().solve(MatrixXd::Identity(task_dim, task_dim));

        base_task_Jac_inv_T[name].setZero(task_dim, MODEL_DOF_VIRTUAL);
        base_task_Jac_inv_T[name] = base_task_lambda[name] * J_task * M_inv_ * base_contact_N;

        base_task_Jac_inv_T_S_T[name].setZero(task_dim, MODEL_DOF);
        base_task_Jac_inv_T_S_T[name] = base_task_Jac_inv_T[name].rightCols(MODEL_DOF);
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

        
        wrench_desired[name] = Eigen::Vector6d::Zero();
    }

    q_des.segment(6, MODEL_DOF)= q_init_des;
    dq_des.setZero(); 
    qdot_des.setZero();
    qddot_des.setZero(); 
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
    task_hierarchy= {
            { {base_link_name,  TaskType::Position}, {base_link_name, TaskType::Orientation} },
            // { {com_name,  TaskType::Position}}
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
    x_desired[base_link_name](1) = DyrosMath::cubic(tick, 0, traj_time * hz_, 
                                                   init_support_ee_pos[base_link_name](1), 
                                                   init_support_ee_pos[base_link_name](1) + pelv_dist, 
                                                   0.0, 0.0);

    dx_desired[base_link_name](1) = DyrosMath::cubicDot(tick, 0, traj_time * hz_, 
                                                       init_support_ee_pos[base_link_name](1), 
                                                       init_support_ee_pos[base_link_name](1) + pelv_dist, 
                                                       0.0, 0.0);

    ddx_desired[base_link_name](1) = DyrosMath::cubicDdot(tick, 0, traj_time * hz_, 
                                                         init_support_ee_pos[base_link_name](1), 
                                                         init_support_ee_pos[base_link_name](1) + pelv_dist, 
                                                         0.0, 0.0);

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

    //--- Data Logging
    dataCC1 << x_desired[base_link_name].transpose()  << " " << support_ee_pos[base_link_name].transpose() << std::endl;
    dataCC2 << x_desired[lhand_link_name].transpose() << " " << support_ee_pos[lhand_link_name].transpose() << std::endl;
    dataCC3 << x_desired[rfoot_link_name].transpose() << " " << support_ee_pos[rfoot_link_name].transpose() << std::endl;
    dataCC4 << DyrosMath::rot2Euler(R_desired[lhand_link_name]).transpose() << " " << DyrosMath::rot2Euler(support_ee_rot[lhand_link_name]).transpose() << std::endl;
    dataCC5 << DyrosMath::rot2Euler(R_desired[base_link_name]).transpose() << " " << DyrosMath::rot2Euler(support_ee_rot[base_link_name]).transpose() << std::endl;

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
    dataCC2 << support_x_desired[lhand_link_name].transpose() << " " << support_ee_pos[lhand_link_name].transpose() << std::endl;
    dataCC3 << support_x_desired[rfoot_link_name].transpose() << " " << support_ee_pos[rfoot_link_name].transpose() << std::endl;
    dataCC4 << DyrosMath::rot2Euler(support_R_desired[lhand_link_name]).transpose() << " " << DyrosMath::rot2Euler(support_ee_rot[lhand_link_name]).transpose() << std::endl;

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
    dataCC2 << x_desired[lhand_link_name].transpose() << " " << support_ee_pos[lhand_link_name].transpose() << std::endl;
    dataCC3 << x_desired[rfoot_link_name].transpose() << " " << support_ee_pos[rfoot_link_name].transpose() << std::endl;
    dataCC4 << DyrosMath::rot2Euler(R_desired[lhand_link_name]).transpose() << " " << DyrosMath::rot2Euler(support_ee_rot[lhand_link_name]).transpose() << std::endl;
    dataCC5 << DyrosMath::rot2Euler(R_desired[base_link_name]).transpose() << " " << DyrosMath::rot2Euler(support_ee_rot[base_link_name]).transpose() << std::endl;

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
        std::cout << "tick: " << tick << std::endl;
        is_left_contact_transition = true;  // Update state transition next tick
    }
}

//--- Joy Utils
void CustomController::xBoxJoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double vel_threshold = 0.1;

    target_vel_x_   = DyrosMath::minmax_cut(joy->axes[1] * vel_threshold, -vel_threshold, vel_threshold);
    target_vel_y_   = DyrosMath::minmax_cut(joy->axes[0] * vel_threshold, -vel_threshold, vel_threshold);
    target_vel_yaw_ = DyrosMath::minmax_cut(joy->axes[3] * vel_threshold, -vel_threshold, vel_threshold);
}

