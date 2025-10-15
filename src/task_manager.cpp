#include "task_manager.h"

using namespace TOCABI;

TaskManager::TaskManager(RobotData& rd) : rd_(rd)
{
    for (int idx = 0; idx < LINK_NUMBER; idx++)
    {
        rd_.link_[idx].x_desired.setZero();
        rd_.link_[idx].rot_desired.setIdentity();
    }
}

void TaskManager::setControlFrequency(double &hz)
{
    hz_ = hz;
}

void TaskManager::runTestMotion(const TaskMotionType& motion_mode, const double &traj_time, const double &pelv_dist, const double &hand_dist, const double &foot_height, const double &step_duration)
{
    switch (motion_mode)
    {
        case TaskMotionType::PelvHand:
            movePelvHandPose(traj_time, pelv_dist, hand_dist);
            break;
        case TaskMotionType::Taichi:
            moveTaichiMotion(traj_time, pelv_dist, hand_dist, foot_height);
            break;
        case TaskMotionType::Walking:
            bipedalWalkingController(step_duration, foot_height);
            break;
        case TaskMotionType::None:
            break;
        default:
            break;
    }
}

void TaskManager::movePelvHandPose(const double &traj_time, const double &pelv_dist, const double &hand_dist)
{
    //--- Initialization
    static int sim_tick = 0;
    for (int idx = 0; idx < LINK_NUMBER; idx++)
    {
        rd_.link_[idx].x_desired = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj.setIdentity();
    }

    //--- Pelvis Trajectory
    rd_.link_[Pelvis].x_desired = rd_.link_[Pelvis].support_xpos_init;
    rd_.link_[Pelvis].x_desired(1) = rd_.link_[Pelvis].support_xpos_init(1) + pelv_dist;
    rd_.link_[Pelvis].x_desired  = rd_.link_[Pelvis].x_desired - rd_.link_[Pelvis].support_xpos;

    //--- Both Hand Trajectories
    for (int idx = 1; idx < 3; idx++)
    {
        rd_.link_[Left_Hand].x_desired(idx) = rd_.link_[Left_Hand].local_xpos_init(idx) + hand_dist;
        rd_.link_[Right_Hand].x_desired(idx) = rd_.link_[Right_Hand].local_xpos_init(idx) - hand_dist;
    }

    //--- Trajectory Generation
    for (int idx = 0; idx < LINK_NUMBER; idx++)
    {
        rd_.link_[idx].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[idx].local_xpos_init, rd_.link_[idx].x_desired);
    }

    //--- Increment Tick
    sim_tick++;
}

void TaskManager::moveTaichiMotion(const double &traj_time, const double &pelv_dist, const double &hand_dist, const double &foot_height)
{
    //--- Initialization
    static int sim_tick = 0;
    for (int idx = 0; idx < LINK_NUMBER; idx++)
    {
        rd_.link_[idx].x_desired = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj.setIdentity();
    }

    //--- Pelvis Trajectory
    rd_.link_[Pelvis].x_desired = rd_.link_[Pelvis].support_xpos_init;
    rd_.link_[Pelvis].x_desired(1) = rd_.link_[Pelvis].support_xpos_init(1) + pelv_dist;
    rd_.link_[Pelvis].x_desired  = rd_.link_[Pelvis].x_desired - rd_.link_[Pelvis].support_xpos;

    //--- Hand Trajectory
    for (int idx = 1; idx < 3; idx++)
    {
        rd_.link_[Left_Hand].x_desired(idx) = rd_.link_[Left_Hand].local_xpos_init(idx) + hand_dist;
        rd_.link_[Right_Hand].x_desired(idx) = rd_.link_[Right_Hand].local_xpos_init(idx) - hand_dist;
    }

    //--- Swing Foot Trajectory
    rd_.link_[Right_Foot].x_desired(2) = rd_.link_[Right_Foot].local_xpos_init(2) + foot_height;

    //--- Trajectory Generation
    for (int idx = 0; idx < LINK_NUMBER; idx++)
    {
        rd_.link_[idx].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[idx].local_xpos_init, rd_.link_[idx].x_desired);
    }

    rd_.link_[Right_Foot].SetTrajectoryQuintic(sim_tick, traj_time * hz_, 2.0 * traj_time * hz_, rd_.link_[Right_Foot].local_xpos_init, rd_.link_[Right_Foot].x_desired);

    //--- Increment Tick
    sim_tick++;

    //--- contact transition
    bool local_LF_contact = rd_.ee_[0].contact;
    bool local_RF_contact = rd_.ee_[1].contact;
    if (sim_tick == traj_time * hz_ - 1)
    {
        if (local_LF_contact == true && local_RF_contact == true)
        {
            rd_.is_left_contact_transition = true;
            rd_.is_right_contact_transition = false;
        }
        else
        {
            ROS_ERROR("CONTACT MISSING");
            assert((local_LF_contact == true && local_RF_contact == true)
                || (local_LF_contact == true && local_RF_contact != true)
                || (local_LF_contact != true && local_RF_contact == true));
        }
    }
}

void TaskManager::bipedalWalkingController(const double &step_duration_, const double &foot_height_)
{
    static WalkingManager wm_(rd_); 
    
    //--- Tick Counter
    static int step_tick = 0;   // tick for one step
    static int step_cnt  = 0;   // step count
    const double step_duration = step_duration_;
    const double transition_duration = 2.0;

    bool local_LF_contact = rd_.ee_[0].contact;
    bool local_RF_contact = rd_.ee_[1].contact;
    double trajectory_duration =  (local_LF_contact == true && local_RF_contact == true) ? transition_duration : step_duration;

    wm_.updateSupportInitialState();
    wm_.updateContactState(local_LF_contact, local_RF_contact);
    wm_.getTimeInformation(step_tick, static_cast<int>(trajectory_duration * hz_));
    wm_.computeWalkingMotion(step_cnt, 0.0, 0.0, 0.0, foot_height_);
    step_tick = wm_.updateStepTick(step_cnt);
    std::cout << "step Tick: " << step_tick << " / " << trajectory_duration * hz_ << " , step Cnt: " << step_cnt << std::endl;
}