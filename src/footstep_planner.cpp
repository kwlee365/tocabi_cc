#include "footstep_planner.h"

FootstepPlanner::FootstepPlanner() {}

Eigen::Vector2d FootstepPlanner::planFootstep(const Eigen::Vector3d& current_hip_pos,
                            const Eigen::Vector3d& current_base_vel,
                            const double& step_duration,
                            const double& vx,
                            const double& vy,
                            const double& wz)
{
    // --- User-defined parameters (Table I) ---
    double t_stance = step_duration;              // total step duration [s]
    double h = 0.73;                               // base height (m)
    double g = GRAVITY;                           // gravity
    double omega = std::sqrt(g/h);

    // --- current hip pos
    Eigen::Vector3d p_hip; p_hip.setZero();
    p_hip = current_hip_pos;

    // --- Raibert Heuristic
    Eigen::Vector3d v_cmd; v_cmd.setZero();
    v_cmd(0) = vx; v_cmd(1) = 0.0; v_cmd(2) = 0.0; 
    Eigen::Vector3d v; v.setZero();
    v = current_base_vel;
    double k = 0.03;

    Eigen::Vector3d p_symmetry; p_symmetry.setZero();
    p_symmetry = (t_stance / 2.0) * v + k * (v - v_cmd);

    // --- Centrifugal term
    Eigen::Vector3d w_cmd; w_cmd.setZero();
    w_cmd(0) = 0.0; w_cmd(1) = 0.0; w_cmd(2) = wz;

    Eigen::Vector3d p_centrifugal; p_centrifugal.setZero();
    p_centrifugal = (1.0 / (2.0 * omega)) * v.cross(w_cmd);

    // --- Desired footstep location
    Eigen::Vector2d p_total; p_total.setZero();
    p_total = (p_hip + p_symmetry + p_centrifugal).head(2);
    p_total(0) = p_hip(0);
    p_total(1) = p_hip(1);

    return p_total;
}