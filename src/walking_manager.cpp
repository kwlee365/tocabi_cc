#include "walking_manager.h"

ofstream dataWM1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM1.txt");
ofstream dataWM2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM2.txt");
ofstream dataWM3("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM3.txt");

WalkingManager::WalkingManager(RobotData &rd) : rd_(rd)
{

}

void WalkingManager::computeWalkingMotion()
{
    updateSupportInitialState();

    step_command.setZero();
    step_command << step_length, step_width;
    step_command = rotZaxis2d(foot_yaw_angle) * step_command;

    step_length = step_command(0);
    step_width  = step_command(1);

    calcFootstepQueue();
    calcCapturePointQueue();
    getFootTrajectory();
    getComTrajectory();

    updateStepTick();
}

void WalkingManager::calcFootstepQueue()
{
    int foot_contact_idx = local_LF_contact ? -1 : +1;
    static bool is_footstep_queue_init = true;

    if (!(local_LF_contact == true && local_RF_contact == true)) // DSP
    {
        if (is_footstep_queue_init == true)
        {
            step_queue.clear();

            Eigen::Vector2d step_command_local; step_command_local.setZero();
            for (int i = 0; i < preview_idx; i++)
            {
                double step_length_local = (i == 0) ? (step_length * 0.5) : step_length;

                step_command_local += Eigen::Vector2d(step_length_local, foot_contact_idx * step_width);
                step_queue.push_back(step_command_local);

                foot_contact_idx *= -1;
            }

            is_footstep_queue_init = false;
        }
        else
        {
            if(is_footstep_update == true)
            {
                Eigen::Vector2d step_command_first = step_queue.front();

                for (auto iter = step_queue.begin(); iter != step_queue.end(); iter++)
                {
                    *iter -= step_command_first;
                }

                step_queue.pop_front();

                Eigen::Vector2d step_command_back = step_queue.back();
                Eigen::Vector2d step_command_new = step_command_back + Eigen::Vector2d(step_length, foot_contact_idx * step_width);

                step_queue.push_back(step_command_new);

                is_footstep_update = false;
            }
        }
    }
}

void WalkingManager::calcCapturePointQueue()
{
    static bool is_cp_queue_init = true;
    if (is_cp_queue_init == true)
    {
        cp_end_queue.clear();

        for (int i = 0; i < preview_idx; i++)
        {
            cp_end_queue.push_back(Eigen::Vector2d(0.0, 0.0));
        }

        is_cp_eos_update = true;
        is_cp_queue_init = false;
    }

    if (!(local_LF_contact == true && local_RF_contact == true)) // DSP
    {
        if (is_cp_eos_update == true)
        {
            cp_end_queue.back() = step_queue.back();

            for (int i = preview_idx - 1; i > 0; --i)
            {
                Eigen::Vector2d cp_end = cp_end_queue[i];
                Eigen::Vector2d zmp = step_queue[i - 1];

                cp_end_queue[i - 1] = zmp + exp(-wn * trajectory_duration) * (cp_end - zmp);
            }

            is_cp_eos_update = false;
        }
    }
}

void WalkingManager::getFootTrajectory()
{
    int support_foot_link_idx, support_hip_link_idx, swing_foot_link_idx, swing_hip_link_idx;

    support_foot_link_idx = local_LF_contact ? Left_Foot  : Right_Foot;
    swing_foot_link_idx   = local_LF_contact ? Right_Foot : Left_Foot;
    support_hip_link_idx  = local_LF_contact ? 5  : 11;
    swing_hip_link_idx    = local_LF_contact ? 11 : 5;

    //--- Swing & Support Feet Trajectory
    footstep_des.setZero(); 
    footstep_des = step_queue.front();
    if (local_LF_contact == true && local_RF_contact == true) // DSP
    {
        rd_.link_[Left_Foot].x_traj = rd_.link_[Left_Foot].support_xpos_init;
        rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].support_xpos_init;

        rd_.link_[Left_Foot].r_traj.setIdentity();
        rd_.link_[Right_Foot].r_traj.setIdentity();
    }
    else // SSP
    {
        rd_.link_[support_foot_link_idx].x_traj = rd_.link_[support_foot_link_idx].support_xpos_init;

        rd_.link_[swing_foot_link_idx].x_traj(0) = cubicBezierPolynomial(step_time, 0.0, trajectory_duration, rd_.link_[swing_foot_link_idx].support_xpos_init(0), rd_.link_[swing_hip_link_idx].support_xpos(0) - 0.07, footstep_des(0));
        rd_.link_[swing_foot_link_idx].x_traj(1) = cubicBezierPolynomial(step_time, 0.0, trajectory_duration, rd_.link_[swing_foot_link_idx].support_xpos_init(1), rd_.link_[swing_hip_link_idx].support_xpos(1), footstep_des(1));
        rd_.link_[swing_foot_link_idx].x_traj(2) = cubicBezierPolynomial(step_time, 0.0, trajectory_duration, rd_.link_[swing_foot_link_idx].support_xpos_init(2), foot_height, 0.0);

        rd_.link_[Left_Foot].r_traj.setIdentity();
        rd_.link_[Right_Foot].r_traj.setIdentity();
    }

    dataWM1 << std::setprecision(3)
            << rd_.link_[Left_Foot].x_traj(0) << "," << rd_.link_[Left_Foot].x_traj(1) << "," << rd_.link_[Left_Foot].x_traj(2) << ","
            << rd_.link_[Left_Foot].support_xpos(0) << "," << rd_.link_[Left_Foot].support_xpos(1) << "," << rd_.link_[Left_Foot].support_xpos(2)
            << std::endl;

    dataWM2 << std::setprecision(3)
            << rd_.link_[Right_Foot].x_traj(0) << "," << rd_.link_[Right_Foot].x_traj(1) << "," << rd_.link_[Right_Foot].x_traj(2) << ","
            << rd_.link_[Right_Foot].support_xpos(0) << "," << rd_.link_[Right_Foot].support_xpos(1) << "," << rd_.link_[Right_Foot].support_xpos(2)
            << std::endl;

    rd_.link_[Left_Foot].x_traj  = rd_.link_[Left_Foot].x_traj - rd_.link_[Pelvis].support_xpos;
    rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].x_traj - rd_.link_[Pelvis].support_xpos;
}

void WalkingManager::getComTrajectory()
{
    //--- Base Test
    if (local_LF_contact == true && local_RF_contact == true) // DSP
    {
        rd_.link_[Pelvis].x_desired =  rd_.link_[Pelvis].support_xpos_init;
        rd_.link_[Pelvis].x_desired(1) =  rd_.link_[Pelvis].support_xpos_init(1) + 0.03;
        rd_.link_[Pelvis].x_desired(2) = 0.765;
    }
    else
    {
        int foot_contact_idx = local_LF_contact ? +1 : -1;

        rd_.link_[Pelvis].x_desired(0) = footstep_des(0) / 2.0 - 0.05;
        rd_.link_[Pelvis].x_desired(1) = footstep_des(1) / 2.0 + foot_contact_idx * 0.01;
        rd_.link_[Pelvis].x_desired(2) = 0.765;
    }

    rd_.link_[Pelvis].SetTrajectoryQuintic(step_time, 0.0, trajectory_duration, rd_.link_[Pelvis].support_xpos_init, rd_.link_[Pelvis].x_desired);


    dataWM3 << std::setprecision(3)
            << rd_.link_[Pelvis].x_traj(0) << "," << rd_.link_[Pelvis].x_traj(1) << "," << rd_.link_[Pelvis].x_traj(2) << ","
            << rd_.link_[Pelvis].support_xpos(0) << "," << rd_.link_[Pelvis].support_xpos(1) << "," << rd_.link_[Pelvis].support_xpos(2)
            << std::endl;

    rd_.link_[Pelvis].x_traj = rd_.link_[Pelvis].x_traj - rd_.link_[Pelvis].support_xpos;
    rd_.link_[Pelvis].r_traj.setIdentity();
}

//--- State Initialization
void WalkingManager::updateSupportInitialState()
{
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_traj = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj = rd_.link_[idx].local_rotm_init;
    }

    if (is_support_transition == true)
    {
        for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
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

//--- Tick Update
void WalkingManager::updateStepTick()
{
    step_tick++;

    static bool is_transfer_phase = true;
    if (is_transfer_phase == true)
    {
        if (step_tick >= static_cast<int>(trajectory_duration * hz_))
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

            is_transfer_phase = false;
        }
    }
    else
    {
        if (step_tick >= static_cast<int>(trajectory_duration * hz_))
        {
            if (local_LF_contact != true && local_RF_contact == true)
            {
                rd_.is_left_contact_transition = true;

                is_support_transition = true;
                is_footstep_update = true;
                is_cp_eos_update = true;
            }
            else if (local_LF_contact == true && local_RF_contact != true)
            {
                rd_.is_right_contact_transition = true;

                is_support_transition = true;
                is_footstep_update = true;
                is_cp_eos_update = true;
            }
            else
            {
                ROS_ERROR("Contact Indicator are assigned with something wrong value.");
            }

            step_tick = 0;
            step_cnt++;
        }
    }
}


//--- Class Setter
void WalkingManager::setControlFrequency(const double &hz)
{
    hz_ = hz;
}

void WalkingManager::setCenterOfMassHeight(const double &com_height_)
{
    com_height = com_height_;
    wn = sqrt(GRAVITY / com_height);
}

void WalkingManager::setTransferDuration(const double& transfer_duration_)
{
    transfer_duration = transfer_duration_;
}

void WalkingManager::updateContactState(const bool &local_LF_contact_, const bool &local_RF_contact_)
{
    local_LF_contact = local_LF_contact_;
    local_RF_contact = local_RF_contact_;
}

void WalkingManager::setWalkingParameter(const double &step_length_, const double &foot_yaw_angle_, const double &foot_height_)
{
    step_length = step_length_;
    step_width     = 0.22;
    foot_yaw_angle = foot_yaw_angle_;
    foot_height = foot_height_;
}

void WalkingManager::setStepDuration(const double& step_duration_)
{
    step_duration = step_duration_;

    trajectory_duration =  (local_LF_contact == true && local_RF_contact == true) ? transfer_duration : step_duration;

    step_time = static_cast<double>(step_tick) / hz_;
}