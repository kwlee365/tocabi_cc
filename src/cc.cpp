#include "cc.h"

using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd), kin_wbc_(MODEL_DOF_VIRTUAL)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    ControlVal_.setZero();

    // Load Robot Model
    std::string urdf_path, desc_package_path;
    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);

    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_, true, false);
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
            // when calculating the nullspace projector, 
            // contact jacobian is considered; thus, 
            // do not include the support foot jacobian into the 

            task_hierarchy= {
// /* Hierachy 0: */    { {"COM_id", TaskType::Position} },
/* Hierachy 1: */    { {"Pelvis_Link", TaskType::Position}, {"Pelvis_Link", TaskType::Orientation} },
// /* Hierachy 2: */    { {"L_Foot_Link", TaskType::Position}, {"L_Foot_Link", TaskType::Orientation}, {"R_Foot_Link", TaskType::Position}, {"R_Foot_Link", TaskType::Orientation} }    
/* Hierachy 3: */    { {"Upperbody_Link", TaskType::Orientation} },
/* Hierachy 4: */    { {"Head_Link", TaskType::Orientation} },
/* Hierachy 5: */    { {"L_Wrist2_Link", TaskType::Position}, {"L_Wrist2_Link", TaskType::Orientation}, {"R_Wrist2_Link", TaskType::Position}, {"R_Wrist2_Link", TaskType::Orientation} }
            };

            cout << "COMPUTESLOW MODE 7 IS NOW INITIALIZED" << endl;
            cout << "TIME: "<< rd_.control_time_ << endl; 

            is_mode_7_init = false;
        }

        std::map<std::string, Eigen::Matrix3d> R_desired;
        std::map<std::string, Eigen::Vector3d> x_desired, dx_desired, ddx_desired, v_desired, dv_desired;

        for (const auto& [name, idx] : link_index_map)
        {
            x_desired[name]   = init_base_ee_pos[name];
            dx_desired[name]  = Eigen::Vector3d::Zero();
            ddx_desired[name] = Eigen::Vector3d::Zero();

            R_desired[name]   = init_base_ee_rot[name];
            v_desired[name]   = Eigen::Vector3d::Zero();
            dv_desired[name]  = Eigen::Vector3d::Zero();

            rd_.q_desired = q_init_des;
        }

        // Test
        static int tick = 0;
        x_desired["Pelvis_Link"](1) = DyrosMath::cubic(tick, 0, 8000, 
                                                         init_base_ee_pos["Pelvis_Link"](1), 
                                                         init_base_ee_pos["Pelvis_Link"](1) - 0.2, 
                                                         0.0, 0.0);

        dx_desired["Pelvis_Link"](1) = DyrosMath::cubicDot(tick, 0, 8000, 
                                                           init_base_ee_pos["Pelvis_Link"](1), 
                                                           init_base_ee_pos["Pelvis_Link"](1) - 0.2, 
                                                           0.0, 0.0);

        ddx_desired["Pelvis_Link"](1) = DyrosMath::cubicDdot(tick, 0, 8000, 
                                                             init_base_ee_pos["Pelvis_Link"](1), 
                                                             init_base_ee_pos["Pelvis_Link"](1) - 0.2, 
                                                             0.0, 0.0);

        // x_desired["L_Wrist2_Link"](2) = DyrosMath::cubic(tick, 0, 8000, 
        //                                                  init_base_ee_pos["L_Wrist2_Link"](2), 
        //                                                  init_base_ee_pos["L_Wrist2_Link"](2) + 0.2, 
        //                                                  0.0, 0.0);

        // dx_desired["L_Wrist2_Link"](2) = DyrosMath::cubicDot(tick, 0, 8000, 
        //                                                      init_base_ee_pos["L_Wrist2_Link"](2), 
        //                                                      init_base_ee_pos["L_Wrist2_Link"](2) + 0.2, 
        //                                                      0.0, 0.0);

        // ddx_desired["L_Wrist2_Link"](2) = DyrosMath::cubicDdot(tick, 0, 8000, 
        //                                                        init_base_ee_pos["L_Wrist2_Link"](2), 
        //                                                        init_base_ee_pos["L_Wrist2_Link"](2) + 0.2, 
        //                                                        0.0, 0.0);

        // x_desired["R_Wrist2_Link"](2) = DyrosMath::cubic(tick, 0, 8000, 
        //                                                  init_base_ee_pos["R_Wrist2_Link"](2), 
        //                                                  init_base_ee_pos["R_Wrist2_Link"](2) - 0.2, 
        //                                                  0.0, 0.0);

        // dx_desired["R_Wrist2_Link"](2) = DyrosMath::cubicDot(tick, 0, 8000, 
        //                                                      init_base_ee_pos["R_Wrist2_Link"](2), 
        //                                                      init_base_ee_pos["R_Wrist2_Link"](2) - 0.2, 
        //                                                      0.0, 0.0);

        // ddx_desired["R_Wrist2_Link"](2) = DyrosMath::cubicDdot(tick, 0, 8000, 
        //                                                        init_base_ee_pos["R_Wrist2_Link"](2), 
        //                                                        init_base_ee_pos["R_Wrist2_Link"](2) - 0.2, 
        //                                                        0.0, 0.0);


        tick++;

        static Eigen::VectorXd dq_des, qdot_des, qddot_des;
        kin_wbc_.computeKinematicWBC(task_hierarchy,
                                     x_desired, dx_desired, ddx_desired,
                                     R_desired, v_desired, dv_desired,
                                     base_ee_pos, base_ee_rot,
                                     base_Jac_v, base_Jac_w,
                                     base_contact_Jac,
                                     M_,
                                     dq_des, qdot_des, qddot_des);

        if (is_mode_temp_init == true)
        {
            // std::cout << "base_ee_pos[L_Foot_Link]: " << base_ee_pos["L_Foot_Link"].transpose() << std::endl;
            // std::cout << "base_ee_rot[L_Foot_Link]:\n" << base_ee_rot["L_Foot_Link"] << std::endl;
            // std::cout << "x_desired[L_Foot_Link]: " << x_desired["L_Foot_Link"].transpose() << std::endl;
            // std::cout << "R_desired[L_Foot_Link]:\n" << R_desired["L_Foot_Link"] << std::endl;
            // std::cout << "ee[L_Foot_Link]:\n" << kin_wbc_.orientationError(base_ee_rot["L_Foot_Link"], R_desired["L_Foot_Link"])  << std::endl;

            // std::cout << "base_Jac_v[L_Foot_Link]: " << base_Jac_v["L_Foot_Link"] << std::endl;
            // std::cout << "base_Jac_w[L_Foot_Link]:\n" << base_Jac_w["L_Foot_Link"] << std::endl;

            // std::cout << "base_ee_pos[R_Foot_Link]: " << base_ee_pos["R_Foot_Link"].transpose() << std::endl;
            // std::cout << "base_ee_rot[R_Foot_Link]:\n" << base_ee_rot["R_Foot_Link"] << std::endl;
            // std::cout << "x_desired[R_Foot_Link]: " << x_desired["R_Foot_Link"].transpose() << std::endl;
            // std::cout << "R_desired[R_Foot_Link]:\n" << R_desired["R_Foot_Link"] << std::endl;

            // std::cout << "base_Jac_v[R_Foot_Link]: " << base_Jac_v["R_Foot_Link"] << std::endl;
            // std::cout << "base_Jac_w[R_Foot_Link]:\n" << base_Jac_w["R_Foot_Link"] << std::endl;
            // std::cout << "ee[R_Foot_Link]:\n" << kin_wbc_.orientationError(base_ee_rot["R_Foot_Link"], R_desired["R_Foot_Link"])  << std::endl;
            is_mode_temp_init = false;
        }


        rd_.q_desired += dq_des.tail(MODEL_DOF);

        Eigen::VectorQd torque_unbound; torque_unbound.setZero();
        Eigen::VectorQd torque_bound;   torque_bound.setZero();
        torque_unbound = (Kp_diag * (rd_.q_desired - rd_.q_)) - (Kd_diag * rd_.q_dot_);
        for (int i = 0; i < MODEL_DOF; i++) {
            torque_bound(i) = DyrosMath::minmax_cut(torque_unbound(i), -rd_.torque_limit(i), rd_.torque_limit(i));
        }

        //--- Final Torque Command
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
    Eigen::Matrix3d base_rot = rd_.link_[Pelvis].rotm; 
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
        contact_Jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = Jac_v["L_Foot_Link"]; 
        contact_Jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = Jac_w["L_Foot_Link"]; 
        contact_Jac.block(6, 0, 3, MODEL_DOF_VIRTUAL) = Jac_v["R_Foot_Link"]; 
        contact_Jac.block(9, 0, 3, MODEL_DOF_VIRTUAL) = Jac_w["R_Foot_Link"]; 

        //--- Base frame
        base_Jac_v[name]  = base_rot.transpose() * Jac_v[name];
        base_Jac_w[name]  = base_rot.transpose() * Jac_w[name];
        base_ee_pos[name] = base_rot.transpose() * (ee_pos[name] - base_pos);
        base_ee_rot[name] = base_rot.transpose() *  ee_rot[name];                             
        base_ee_v[name]   = base_rot.transpose() *  ee_v[name];                               
        base_ee_w[name]   = base_rot.transpose() *  ee_w[name]; 
        base_contact_Jac.block(0, 0, 3, MODEL_DOF_VIRTUAL) = base_Jac_v["L_Foot_Link"]; 
        base_contact_Jac.block(3, 0, 3, MODEL_DOF_VIRTUAL) = base_Jac_w["L_Foot_Link"]; 
        base_contact_Jac.block(6, 0, 3, MODEL_DOF_VIRTUAL) = base_Jac_v["R_Foot_Link"]; 
        base_contact_Jac.block(9, 0, 3, MODEL_DOF_VIRTUAL) = base_Jac_w["R_Foot_Link"];
    }

    Eigen::VectorQVQd base_q_virtual_;
    base_q_virtual_.segment(0,3) = base_ee_pos["Pelvis_Link"];
    
    Quaterniond base_quat(base_ee_rot["Pelvis_Link"]);
    base_quat.normalize();
    
    base_q_virtual_(3)  = base_quat.x();
    base_q_virtual_(4)  = base_quat.y();
    base_q_virtual_(5)  = base_quat.z();
    base_q_virtual_(39) = base_quat.w();
    
    base_q_virtual_.segment(6, MODEL_DOF) = rd_.q_;

    //--- Dynamics
    M_.setZero(); G_.setZero();
    M_temp_.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL); G_temp_.setZero(MODEL_DOF_VIRTUAL);

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, base_q_virtual_, M_temp_, false);
    M_ = M_temp_;
    RigidBodyDynamics::NonlinearEffects(model_, base_q_virtual_, Eigen::VectorXd::Zero(MODEL_DOF_QVIRTUAL), G_temp_);
    G_ = G_temp_;
}

void CustomController::saveInitialState()
{
    init_base_ee_pos = base_ee_pos;
    init_base_ee_rot = base_ee_rot;
    init_base_ee_v = base_ee_v;
    init_base_ee_w = base_ee_w;
}