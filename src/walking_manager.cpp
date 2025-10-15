#include "walking_manager.h"

ofstream dataWM1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM1.txt");
ofstream dataWM2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM2.txt");
ofstream dataWM3("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM3.txt");

WalkingManager::WalkingManager(RobotData &rd) : rd_(rd)
{
}

void WalkingManager::updateSupportInitialState()
{
    for (int idx = 0; idx < LINK_NUMBER; idx++)
    {
        rd_.link_[idx].x_traj = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj = rd_.link_[idx].local_rotm_init;
    }

    if (is_support_transition == true)
    {
        for (int idx = 0; idx < LINK_NUMBER; idx++)
        {
            //--- Support frame
            rd_.link_[idx].support_xpos_init = rd_.link_[idx].support_xpos;
            rd_.link_[idx].support_rotm_init = rd_.link_[idx].support_rotm;
            rd_.link_[idx].support_v_init = rd_.link_[idx].support_v;
            rd_.link_[idx].support_w_init = rd_.link_[idx].support_w;
        }

        is_support_transition = false;
    }
    else
    {
        // Do nothing
    }
}

void WalkingManager::updateContactState(const bool &local_LF_contact_, const bool &local_RF_contact_)
{
    local_LF_contact = local_LF_contact_;
    local_RF_contact = local_RF_contact_;
}

void WalkingManager::getTimeInformation(const int &step_tick_, const int &trajectory_duration_)
{
    step_tick = step_tick_;
    trajectory_duration = trajectory_duration_;
}

int WalkingManager::updateStepTick(int &step_cnt)
{
    step_tick++;

    static bool is_transfer_phase = true;
    if (is_transfer_phase == true)
    {
        if (step_tick >= trajectory_duration)
        {
            if (local_LF_contact == true && local_RF_contact == true)
            {
                rd_.is_left_contact_transition = true;
                is_support_transition = true;
            }
            else
            {
                ROS_ERROR("Contact Indicator are assigned with something wrong value.");
                assert(local_LF_contact == true && local_RF_contact == true);
            }

            step_tick = 0;
            step_cnt++;

            is_transfer_phase = false;
        }
    }
    else
    {
        if (step_tick >= trajectory_duration - 1)
        {
            if (local_LF_contact != true && local_RF_contact == true)
            {
                rd_.is_left_contact_transition = true;
                is_support_transition = true;
            }
            else if (local_LF_contact == true && local_RF_contact != true)
            {
                rd_.is_right_contact_transition = true;
                is_support_transition = true;
            }
            else
            {
                ROS_ERROR("Contact Indicator are assigned with something wrong value.");
            }

            step_tick = 0;
            step_cnt++;
        }
    }

    return (step_tick);
}

void WalkingManager::computeWalkingMotion(const int &step_cnt, const double &step_length_x_, const double &step_length_y_, const double &step_length_yaw_, const double &foot_height)
{
    const int preview_idx = 3;

    step_length_x = (step_cnt == 0) ? (step_length_x_ * 0.5) : step_length_x_;
    step_length_y = step_length_y_;
    step_length_y = step_length_yaw_;

    getFootTrajectory(foot_height);
    getPelvTrajectory();
}

void WalkingManager::getFootTrajectory(const double &foot_height)
{
    int support_foot_link_idx, support_hip_link_idx, swing_foot_link_idx, swing_hip_link_idx;

    support_foot_link_idx = local_LF_contact ? Left_Foot  : Right_Foot;
    swing_foot_link_idx   = local_LF_contact ? Right_Foot : Left_Foot;

    //--- Desired Hip Pos
    footstep_des.setZero();

    //--- Swing & Support Feet Trajectory
    if (local_LF_contact == true && local_RF_contact == true) // DSP
    {
        footstep_des = rd_.link_[Pelvis].support_xpos_init.head(2);
        footstep_des(1) += 0.02;

        rd_.link_[Left_Foot].x_traj = rd_.link_[Left_Foot].support_xpos_init;
        rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].support_xpos_init;

        rd_.link_[Left_Foot].r_traj.setIdentity();
        rd_.link_[Right_Foot].r_traj.setIdentity();
    }
    else // SSP
    {
        footstep_des << rd_.link_[swing_foot_link_idx].support_xpos_init(0) + step_length_x,
                        rd_.link_[swing_foot_link_idx].support_xpos_init(1) + step_length_y;

        rd_.link_[support_foot_link_idx].x_traj = rd_.link_[support_foot_link_idx].support_xpos_init;

        rd_.link_[swing_foot_link_idx].x_traj(0) = cubicBezierPolynomial(step_tick, 0.0, trajectory_duration, rd_.link_[swing_foot_link_idx].support_xpos_init(0), (rd_.link_[swing_foot_link_idx].support_xpos_init(0) + footstep_des(0)) / 2.0, footstep_des(0));
        rd_.link_[swing_foot_link_idx].x_traj(1) = cubicBezierPolynomial(step_tick, 0.0, trajectory_duration, rd_.link_[swing_foot_link_idx].support_xpos_init(1), (rd_.link_[swing_foot_link_idx].support_xpos_init(1) + footstep_des(1)) / 2.0, footstep_des(1));
        rd_.link_[swing_foot_link_idx].x_traj(2) = cubicBezierPolynomial(step_tick, 0.0, trajectory_duration, rd_.link_[swing_foot_link_idx].support_xpos_init(2), foot_height, rd_.link_[swing_foot_link_idx].support_xpos_init(2));

        rd_.link_[Left_Foot].r_traj.setIdentity();
        rd_.link_[Right_Foot].r_traj.setIdentity();
    }

    dataWM1 << rd_.link_[Left_Foot].x_traj.transpose() << " " << rd_.link_[Left_Foot].support_xpos.transpose() << std::endl;
    dataWM2 << rd_.link_[Right_Foot].x_traj.transpose() << " " << rd_.link_[Right_Foot].support_xpos.transpose() << std::endl;

    rd_.link_[Left_Foot].x_traj = rd_.link_[Left_Foot].x_traj - rd_.link_[Pelvis].support_xpos;
    rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].x_traj - rd_.link_[Pelvis].support_xpos;
}

void WalkingManager::getPelvTrajectory()
{
    //--- Base Test
    if (local_LF_contact == true && local_RF_contact == true) // DSP
    {
        rd_.link_[Pelvis].x_desired(0) = footstep_des(0);
        rd_.link_[Pelvis].x_desired(1) = footstep_des(1);
    }
    else
    {
        rd_.link_[Pelvis].x_desired(0) = footstep_des(0) / 2.0 - 0.05;
        rd_.link_[Pelvis].x_desired(1) = footstep_des(1) / 2.0;
    }
    rd_.link_[Pelvis].x_desired(2) = 0.765;

    rd_.link_[Pelvis].SetTrajectoryQuintic(step_tick, 0.0, trajectory_duration, rd_.link_[Pelvis].support_xpos_init, rd_.link_[Pelvis].x_desired);
    rd_.link_[Pelvis].x_traj = rd_.link_[Pelvis].x_traj - rd_.link_[Pelvis].support_xpos;
}
