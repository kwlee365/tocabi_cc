#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <filesystem>

using namespace casadi;

// https://groups.google.com/g/casadi-users/c/FNqBF6ilFgc?pli=1

std::string current_path = std::filesystem::current_path().parent_path().parent_path().string();
std::string prefix_code  = current_path + "/wbc_function/";   
std::string prefix_lib   = current_path + "/wbc_lib/";
std::string func_name    = "wbc_func.c";
std::string lib_name     = "lib_wbc_func.so";

const int nv = 39;
const int na = 33;

int main(){
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    /*---------------------------------------------------*/
    /*------------------ (0) Variables ------------------*/
    /*---------------------------------------------------*/

    std::pair<int, int> dims;

    //--- Decision variables (dim = 108)
    SX qddot  = SX::sym("qddot", nv);
    SX torque = SX::sym("torque", na);
    SX wrench_left_hand  = SX::sym("wrench_left_hand",  6);
    SX wrench_right_hand = SX::sym("wrench_right_hand", 6);
    SX wrench_left_foot  = SX::sym("wrench_left_foot",  6);
    SX wrench_right_foot = SX::sym("wrench_right_foot", 6);
    SX wrench_pelv  = SX::sym("wrench_pelv",  6);
    SX wrench_chest = SX::sym("wrench_chest", 3);
    SX wrench_head  = SX::sym("wrench_head",  3);

    SX X;
    X = vertcat(X, qddot);
    X = vertcat(X, torque);
    X = vertcat(X, wrench_left_hand, wrench_right_hand, wrench_left_foot, wrench_right_foot);
    X = vertcat(X, wrench_pelv, wrench_chest, wrench_head);

    //--- System Parameter
    SX H = SX::sym("H", nv, nv);
    SX G = SX::sym("G", nv);
    SX J_left_hand  = SX::sym("J_left_hand",  6, nv);
    SX J_right_hand = SX::sym("J_right_hand", 6, nv);
    SX J_left_foot  = SX::sym("J_left_foot",  6, nv);
    SX J_right_foot = SX::sym("J_right_foot", 6, nv);
    SX J_pelv  = SX::sym("J_pelv",  6, nv);
    SX J_chest = SX::sym("J_chest", 3, nv);
    SX J_head  = SX::sym("J_head",  3, nv);

    SX Jdot_left_foot  = SX::sym("Jdot_left_foot",  6, nv);
    SX Jdot_right_foot = SX::sym("Jdot_right_foot", 6, nv);

    //--- Joint measurements 
    SX q = SX::sym("q", nv);
    SX qdot = SX::sym("qdot", nv);
    SX q_act     = q(Slice(nv - na, nv));          // Only actuated joints
    SX qdot_act  = qdot(Slice(nv - na, nv));       // Only actuated joints
    SX qddot_act = qddot(Slice(nv - na, nv));  

    //--- Cost Functions Parameters
    SX W_wrench_left_hand  = SX::sym("W_wrench_left_hand",  6);
    SX W_wrench_right_hand = SX::sym("W_wrench_right_hand", 6);
    SX W_wrench_left_foot  = SX::sym("W_wrench_left_foot",  6);
    SX W_wrench_right_foot = SX::sym("W_wrench_right_foot", 6);
    SX W_wrench_pelv  = SX::sym("W_wrench_pelv",  6);
    SX W_wrench_chest = SX::sym("W_wrench_chest", 3);
    SX W_wrench_head  = SX::sym("W_wrench_head",  3);

    SX W_torque = SX::sym("W_torque", na);
    SX W_torque_prev = SX::sym("W_torque_prev", na);

    SX torque_prev = SX::sym("torque_prev", na);    // Previous torque for cost function
    SX wrench_left_hand_des  = SX::sym("wrench_left_hand_des",  6);
    SX wrench_right_hand_des = SX::sym("wrench_right_hand_des", 6);
    SX wrench_left_foot_des  = SX::sym("wrench_left_foot_des",  6);
    SX wrench_right_foot_des = SX::sym("wrench_right_foot_des", 6);
    SX wrench_pelv_des  = SX::sym("wrench_pelv_des",  6);
    SX wrench_chest_des = SX::sym("wrench_chest_des", 3);
    SX wrench_head_des  = SX::sym("wrench_head_des",  3);

    //--- Constraints Parameters
    SX fL_x = wrench_left_foot(0);
    SX fL_y = wrench_left_foot(1);
    SX fL_z = wrench_left_foot(2);
    SX mL_x = wrench_left_foot(3);
    SX mL_y = wrench_left_foot(4);
    SX mL_z = wrench_left_foot(5);

    SX fR_x = wrench_right_foot(0);
    SX fR_y = wrench_right_foot(1);
    SX fR_z = wrench_right_foot(2);
    SX mR_x = wrench_right_foot(3);
    SX mR_y = wrench_right_foot(4);
    SX mR_z = wrench_right_foot(5);

    SX mu = SX::sym("mu");  // Friction coefficient

    SX alpha1 = SX::sym("alpha1");  // For CBF constraints
    SX alpha2 = SX::sym("alpha2");
    SX alpha3 = SX::sym("alpha3");

    SX torque_lim  = SX::sym("torque_lim", na);     // For actuation limits
    SX q_pos_l_lim = SX::sym("q_pos_l_lim", na);
    SX q_pos_h_lim = SX::sym("q_pos_h_lim", na);
    SX q_vel_l_lim = SX::sym("q_vel_l_lim", na);
    SX q_vel_h_lim = SX::sym("q_vel_h_lim", na);

    SX f_z_max = SX::sym("f_z_max");
    SX f_z_min = SX::sym("f_z_min");

    SX footX = SX::sym("footX");    // FOOT HEIGHT
    SX footY = SX::sym("footY");    // FOOT WIDTH


    /*-------------------------------------------------------*/
    /*------------------ (1) Cost Function ------------------*/
    /*-------------------------------------------------------*/

    // TODO: || Jbar.transpose() * torque - F_imp ||^2 
    // TODO: Delete impedance term in the dynamics constraints

    SX J_LH = mtimes(transpose(wrench_left_hand  - wrench_left_hand_des) , mtimes(diag(W_wrench_left_hand),  (wrench_left_hand  - wrench_left_hand_des)));
    SX J_RH = mtimes(transpose(wrench_right_hand - wrench_right_hand_des), mtimes(diag(W_wrench_right_hand), (wrench_right_hand - wrench_right_hand_des)));
    SX J_LF = mtimes(transpose(wrench_left_foot  - wrench_left_foot_des) , mtimes(diag(W_wrench_left_foot),  (wrench_left_foot  - wrench_left_foot_des)));
    SX J_RF = mtimes(transpose(wrench_right_foot - wrench_right_foot_des), mtimes(diag(W_wrench_right_foot), (wrench_right_foot - wrench_right_foot_des)));
    SX J_PELV  = mtimes(transpose(wrench_pelv    - wrench_pelv_des),       mtimes(diag(W_wrench_pelv),       (wrench_pelv       - wrench_pelv_des)));
    SX J_CHEST = mtimes(transpose(wrench_chest   - wrench_chest_des),      mtimes(diag(W_wrench_chest),      (wrench_chest      - wrench_chest_des)));
    SX J_HEAD  = mtimes(transpose(wrench_head    - wrench_head_des),       mtimes(diag(W_wrench_head),       (wrench_head       - wrench_head_des)));

    SX J_torque = mtimes(transpose(torque), mtimes(diag(W_torque), torque));
    SX J_torque_prev = mtimes(transpose(torque - torque_prev), mtimes(diag(W_torque_prev), torque - torque_prev));

    SX J = J_LH + J_RH + J_LF + J_RF + J_PELV + J_CHEST + J_HEAD + J_torque + J_torque_prev;
    SX J_v = jacobian(J, X);
    SX J_vv = hessian(J, X);


    /*-----------------------------------------------------*/
    /*------------------ (2) Constraints ------------------*/
    /*-----------------------------------------------------*/

    //--- (2-1) Whole-body dynamics
    SX S_T = SX::zeros(nv, na);
    for (int i = nv - na; i < nv; ++i) S_T(i, i - (nv - na)) = 1.0;
    SX ceq_wb = mtimes(H, qddot) + G + mtimes(transpose(J_left_hand),  wrench_left_hand) 
                                     + mtimes(transpose(J_right_hand), wrench_right_hand) 
                                     + mtimes(transpose(J_left_foot),  wrench_left_foot) 
                                     + mtimes(transpose(J_right_foot), wrench_right_foot) 
                                     + mtimes(transpose(J_pelv),  wrench_pelv) 
                                     + mtimes(transpose(J_chest), wrench_chest) 
                                     + mtimes(transpose(J_head),  wrench_head) 
                                     - mtimes(S_T, torque);
    SX ceq_wb_v = jacobian(ceq_wb, X);

    //--- (2-2) Contact constraints
    SX ceq_cc_leftfoot    = mtimes(J_left_foot, qddot) + mtimes(Jdot_left_foot, qdot);
    SX ceq_cc_leftfoot_v  = jacobian(ceq_cc_leftfoot, X);
    SX ceq_cc_rightfoot   = mtimes(J_right_foot, qddot) + mtimes(Jdot_right_foot, qdot);
    SX ceq_cc_rightfoot_v = jacobian(ceq_cc_rightfoot, X);

    //--- (2-3) Torque constraints
    SX cineq_torque_max =  torque - torque_lim;
    SX cineq_torque_min = -torque - torque_lim;
    SX cineq_torque_max_v = jacobian(cineq_torque_max, X);
    SX cineq_torque_min_v = jacobian(cineq_torque_min, X);

    //--- (2-4) Joint position constraints   
    SX cineq_qpos_max   = qddot_act + (alpha1 + alpha2) * qdot_act + alpha1 * alpha2 * (q_act - q_pos_h_lim);
    SX cineq_qpos_min   =-qddot_act - (alpha1 + alpha2) * qdot_act - alpha1 * alpha2 * (q_act - q_pos_l_lim);
    SX cineq_qpos_max_v = jacobian(cineq_qpos_max, X);
    SX cineq_qpos_min_v = jacobian(cineq_qpos_min, X);

    //--- (2-5) Joint velocity constraints
    SX cineq_qvel_max = qddot_act + (alpha3) * (qdot_act - q_vel_h_lim);
    SX cineq_qvel_min =-qddot_act - (alpha3) * (qdot_act - q_vel_l_lim);
    SX cineq_qvel_max_v = jacobian(cineq_qvel_max, X);
    SX cineq_qvel_min_v = jacobian(cineq_qvel_min, X);

    //--- (2-6) Unilateral constraints
    SX cineq_unilateral_leftfoot_max  = fL_z - f_z_max;
    SX cineq_unilateral_leftfoot_min  =-fL_z + f_z_min; 
    SX cineq_unilateral_rightfoot_max = fR_z - f_z_max;
    SX cineq_unilateral_rightfoot_min =-fR_z + f_z_min;

    SX cineq_unilateral_leftfoot_max_v  = jacobian(cineq_unilateral_leftfoot_max,  X);
    SX cineq_unilateral_leftfoot_min_v  = jacobian(cineq_unilateral_leftfoot_min,  X);
    SX cineq_unilateral_rightfoot_max_v = jacobian(cineq_unilateral_rightfoot_max, X);
    SX cineq_unilateral_rightfoot_min_v = jacobian(cineq_unilateral_rightfoot_min, X);

    //--- (2-7) Friction cone constraints
    SX cineq_fric_leftfoot_max;  SX cineq_fric_leftfoot_min;
    SX cineq_fric_rightfoot_max; SX cineq_fric_rightfoot_min;

    // NO SLIP CONDITION (HORIZONTAL FORCE, X)
    SX cineq_fric_leftfoot_noslip_x_max  = fL_x - mu * fL_z; 
    SX cineq_fric_leftfoot_noslip_x_min  =-fL_x - mu * fL_z; 
    SX cineq_fric_rightfoot_noslip_x_max = fR_x - mu * fR_z;
    SX cineq_fric_rightfoot_noslip_x_min =-fR_x - mu * fR_z;

    // NO SLIP CONDITION (HORIZONTAL FORCE, Y)
    SX cineq_fric_leftfoot_noslip_y_max  = fL_y - mu * fL_z; 
    SX cineq_fric_leftfoot_noslip_y_min  =-fL_y - mu * fL_z; 
    SX cineq_fric_rightfoot_noslip_y_max = fR_y - mu * fR_z;
    SX cineq_fric_rightfoot_noslip_y_min =-fR_y - mu * fR_z;

    // NO TIPPING CONDITION (HORIZONTAL MOMENT, X)
    SX cineq_fric_leftfoot_notipping_x_max  = mL_x - footY * fL_z; 
    SX cineq_fric_leftfoot_notipping_x_min  =-mL_x - footY * fL_z; 
    SX cineq_fric_rightfoot_notipping_x_max = mR_x - footY * fR_z;
    SX cineq_fric_rightfoot_notipping_x_min =-mR_x - footY * fR_z;

    // NO TIPPING CONDITION (HORIZONTAL MOMENT, Y)
    SX cineq_fric_leftfoot_notipping_y_max  = mL_y - footX * fL_z; 
    SX cineq_fric_leftfoot_notipping_y_min  =-mL_y - footX * fL_z; 
    SX cineq_fric_rightfoot_notipping_y_max = mR_y - footX * fR_z;
    SX cineq_fric_rightfoot_notipping_y_min =-mR_y - footX * fR_z;

    cineq_fric_leftfoot_max  = vertcat(cineq_fric_leftfoot_max,  cineq_fric_leftfoot_noslip_x_max,  cineq_fric_leftfoot_noslip_y_max,  cineq_fric_leftfoot_notipping_x_max,  cineq_fric_leftfoot_notipping_y_max);
    cineq_fric_leftfoot_min  = vertcat(cineq_fric_leftfoot_min,  cineq_fric_leftfoot_noslip_x_min,  cineq_fric_leftfoot_noslip_y_min,  cineq_fric_leftfoot_notipping_x_min,  cineq_fric_leftfoot_notipping_y_min);
    cineq_fric_rightfoot_max = vertcat(cineq_fric_rightfoot_max, cineq_fric_rightfoot_noslip_x_max, cineq_fric_rightfoot_noslip_y_max, cineq_fric_rightfoot_notipping_x_max, cineq_fric_rightfoot_notipping_y_max);
    cineq_fric_rightfoot_min = vertcat(cineq_fric_rightfoot_min, cineq_fric_rightfoot_noslip_x_min, cineq_fric_rightfoot_noslip_y_min, cineq_fric_rightfoot_notipping_x_min, cineq_fric_rightfoot_notipping_y_min);

    SX cineq_fric_leftfoot_max_v  = jacobian(cineq_fric_leftfoot_max, X);
    SX cineq_fric_leftfoot_min_v  = jacobian(cineq_fric_leftfoot_min, X);
    SX cineq_fric_rightfoot_max_v = jacobian(cineq_fric_rightfoot_max, X);
    SX cineq_fric_rightfoot_min_v = jacobian(cineq_fric_rightfoot_min, X);


    /*--------------------------------------------------------------------*/
    /*------------------ (3) CasADi Function Generation ------------------*/
    /*--------------------------------------------------------------------*/    
    
    Function J_v_func("J_v_func",
        {qddot, torque, wrench_left_hand, wrench_right_hand, wrench_left_foot, wrench_right_foot, wrench_pelv, wrench_chest, wrench_head,
         wrench_left_hand_des, wrench_right_hand_des, wrench_left_foot_des, wrench_right_foot_des, wrench_pelv_des, wrench_chest_des, wrench_head_des, torque_prev,
         W_wrench_left_hand, W_wrench_right_hand, W_wrench_left_foot, W_wrench_right_foot, W_wrench_pelv, W_wrench_chest, W_wrench_head, W_torque, W_torque_prev},
        {J_v}
    );

    Function J_vv_func("J_vv_func",
        {qddot, torque, wrench_left_hand, wrench_right_hand, wrench_left_foot, wrench_right_foot, wrench_pelv, wrench_chest, wrench_head,
         wrench_left_hand_des, wrench_right_hand_des, wrench_left_foot_des, wrench_right_foot_des, wrench_pelv_des, wrench_chest_des, wrench_head_des, torque_prev,
         W_wrench_left_hand, W_wrench_right_hand, W_wrench_left_foot, W_wrench_right_foot, W_wrench_pelv, W_wrench_chest, W_wrench_head, W_torque, W_torque_prev},
        {J_vv}
    );

    //--- Whole-body dynamics constraints
    Function ceq_wb_func("ceq_wb_func",     {qddot, torque, wrench_left_hand, wrench_right_hand, wrench_left_foot, wrench_right_foot, wrench_pelv, wrench_chest, wrench_head,
                                             H, G, J_left_hand, J_right_hand, J_left_foot, J_right_foot, J_pelv, J_chest, J_head}, 
                                            {ceq_wb});
    Function ceq_wb_v_func("ceq_wb_v_func", {qddot, torque, wrench_left_hand, wrench_right_hand, wrench_left_foot, wrench_right_foot, wrench_pelv, wrench_chest, wrench_head,
                                             H, G, J_left_hand, J_right_hand, J_left_foot, J_right_foot, J_pelv, J_chest, J_head}, 
                                            {ceq_wb_v});

    //--- Contact constraints
    Function ceq_cc_leftfoot_func("ceq_cc_leftfoot_func",       {qddot, qdot, J_left_foot, Jdot_left_foot},   {ceq_cc_leftfoot});
    Function ceq_cc_rightfoot_func("ceq_cc_rightfoot_func",     {qddot, qdot, J_right_foot, Jdot_right_foot}, {ceq_cc_rightfoot});
    Function ceq_cc_leftfoot_v_func("ceq_cc_leftfoot_v_func",   {qddot, qdot, J_left_foot, Jdot_left_foot},   {ceq_cc_leftfoot_v});
    Function ceq_cc_rightfoot_v_func("ceq_cc_rightfoot_v_func", {qddot, qdot, J_right_foot, Jdot_right_foot}, {ceq_cc_rightfoot_v});

    //--- Torque constraints
    Function cineq_torque_max_func("cineq_torque_max_func",     {torque, torque_lim}, {cineq_torque_max});
    Function cineq_torque_min_func("cineq_torque_min_func",     {torque, torque_lim}, {cineq_torque_min});
    Function cineq_torque_max_v_func("cineq_torque_max_v_func", {torque, torque_lim}, {cineq_torque_max_v});
    Function cineq_torque_min_v_func("cineq_torque_min_v_func", {torque, torque_lim}, {cineq_torque_min_v});

    //--- Joint position constraints
    Function cineq_qpos_max_func("cineq_qpos_max_func",     {qddot, q, qdot, q_pos_h_lim, alpha1, alpha2}, {cineq_qpos_max});
    Function cineq_qpos_min_func("cineq_qpos_min_func",     {qddot, q, qdot, q_pos_l_lim, alpha1, alpha2}, {cineq_qpos_min});
    Function cineq_qpos_max_v_func("cineq_qpos_max_v_func", {qddot, q, qdot, q_pos_h_lim, alpha1, alpha2}, {cineq_qpos_max_v});
    Function cineq_qpos_min_v_func("cineq_qpos_min_v_func", {qddot, q, qdot, q_pos_l_lim, alpha1, alpha2}, {cineq_qpos_min_v});

    //--- Joint velocity constraints
    Function cineq_qvel_max_func("cineq_qvel_max_func",     {qddot, qdot, q_vel_h_lim, alpha3}, {cineq_qvel_max});
    Function cineq_qvel_min_func("cineq_qvel_min_func",     {qddot, qdot, q_vel_l_lim, alpha3}, {cineq_qvel_min});
    Function cineq_qvel_max_v_func("cineq_qvel_max_v_func", {qddot, qdot, q_vel_h_lim, alpha3}, {cineq_qvel_max_v});
    Function cineq_qvel_min_v_func("cineq_qvel_min_v_func", {qddot, qdot, q_vel_l_lim, alpha3}, {cineq_qvel_min_v});

    //--- Unilateral constraints
    Function cineq_unilateral_leftfoot_max_func("cineq_unilateral_leftfoot_max_func",   {wrench_left_foot,  f_z_max}, {cineq_unilateral_leftfoot_max});
    Function cineq_unilateral_leftfoot_min_func("cineq_unilateral_leftfoot_min_func",   {wrench_left_foot,  f_z_min}, {cineq_unilateral_leftfoot_min});
    Function cineq_unilateral_rightfoot_max_func("cineq_unilateral_rightfoot_max_func", {wrench_right_foot, f_z_max}, {cineq_unilateral_rightfoot_max});
    Function cineq_unilateral_rightfoot_min_func("cineq_unilateral_rightfoot_min_func", {wrench_right_foot, f_z_min}, {cineq_unilateral_rightfoot_min});

    Function cineq_unilateral_leftfoot_max_v_func("cineq_unilateral_leftfoot_max_v_func",   {wrench_left_foot,  f_z_max}, {cineq_unilateral_leftfoot_max_v});
    Function cineq_unilateral_leftfoot_min_v_func("cineq_unilateral_leftfoot_min_v_func",   {wrench_left_foot,  f_z_min}, {cineq_unilateral_leftfoot_min_v});
    Function cineq_unilateral_rightfoot_max_v_func("cineq_unilateral_rightfoot_max_v_func", {wrench_right_foot, f_z_max}, {cineq_unilateral_rightfoot_max_v});
    Function cineq_unilateral_rightfoot_min_v_func("cineq_unilateral_rightfoot_min_v_func", {wrench_right_foot, f_z_min}, {cineq_unilateral_rightfoot_min_v});

    //--- Friction cone constraints
    Function cineq_fric_leftfoot_max_func("cineq_fric_leftfoot_max_func", {wrench_left_foot, mu, footX, footY}, {cineq_fric_leftfoot_max});
    Function cineq_fric_leftfoot_min_func("cineq_fric_leftfoot_min_func", {wrench_left_foot, mu, footX, footY}, {cineq_fric_leftfoot_min});
    Function cineq_fric_rightfoot_max_func("cineq_fric_rightfoot_max_func", {wrench_right_foot, mu, footX, footY}, {cineq_fric_rightfoot_max});
    Function cineq_fric_rightfoot_min_func("cineq_fric_rightfoot_min_func", {wrench_right_foot, mu, footX, footY}, {cineq_fric_rightfoot_min});


    Function cineq_fric_leftfoot_max_v_func("cineq_fric_leftfoot_max_v_func", {wrench_left_foot, mu, footX, footY}, {cineq_fric_leftfoot_max_v});
    Function cineq_fric_leftfoot_min_v_func("cineq_fric_leftfoot_min_v_func", {wrench_left_foot, mu, footX, footY}, {cineq_fric_leftfoot_min_v});
    Function cineq_fric_rightfoot_max_v_func("cineq_fric_rightfoot_max_v_func", {wrench_right_foot, mu, footX, footY}, {cineq_fric_rightfoot_max_v});
    Function cineq_fric_rightfoot_min_v_func("cineq_fric_rightfoot_min_v_func", {wrench_right_foot, mu, footX, footY}, {cineq_fric_rightfoot_min_v});   

    /////////////////////////
    // Function Generation //
    std::cout << "CASADI FUNCTION GENERATION START!!!" << std::endl;
    Dict opts = Dict();
    opts["cpp"] = false; opts["with_header"] = true;    
    CodeGenerator myCodeGen = CodeGenerator(func_name, opts);

    myCodeGen.add(J_v_func);
    myCodeGen.add(J_vv_func);

    myCodeGen.add(ceq_wb_func);
    myCodeGen.add(ceq_wb_v_func);

    myCodeGen.add(ceq_cc_leftfoot_func);
    myCodeGen.add(ceq_cc_leftfoot_v_func);
    myCodeGen.add(ceq_cc_rightfoot_func);
    myCodeGen.add(ceq_cc_rightfoot_v_func);

    myCodeGen.add(cineq_torque_max_func);
    myCodeGen.add(cineq_torque_min_func);
    myCodeGen.add(cineq_torque_max_v_func);
    myCodeGen.add(cineq_torque_min_v_func);

    myCodeGen.add(cineq_qpos_max_func);
    myCodeGen.add(cineq_qpos_min_func);
    myCodeGen.add(cineq_qpos_max_v_func);
    myCodeGen.add(cineq_qpos_min_v_func);

    myCodeGen.add(cineq_qvel_max_func);
    myCodeGen.add(cineq_qvel_min_func);
    myCodeGen.add(cineq_qvel_max_v_func);
    myCodeGen.add(cineq_qvel_min_v_func);

    myCodeGen.add(cineq_unilateral_leftfoot_max_func);
    myCodeGen.add(cineq_unilateral_leftfoot_min_func);
    myCodeGen.add(cineq_unilateral_rightfoot_max_func);
    myCodeGen.add(cineq_unilateral_rightfoot_min_func);

    myCodeGen.add(cineq_unilateral_leftfoot_max_v_func);
    myCodeGen.add(cineq_unilateral_leftfoot_min_v_func);
    myCodeGen.add(cineq_unilateral_rightfoot_max_v_func);
    myCodeGen.add(cineq_unilateral_rightfoot_min_v_func);

    myCodeGen.add(cineq_fric_leftfoot_max_func);
    myCodeGen.add(cineq_fric_leftfoot_min_func);
    myCodeGen.add(cineq_fric_rightfoot_max_func);
    myCodeGen.add(cineq_fric_rightfoot_min_func);

    myCodeGen.add(cineq_fric_leftfoot_max_v_func);
    myCodeGen.add(cineq_fric_leftfoot_min_v_func);
    myCodeGen.add(cineq_fric_rightfoot_max_v_func);
    myCodeGen.add(cineq_fric_rightfoot_min_v_func);

    myCodeGen.generate(prefix_code);

    // compile c code to a shared library
    std::string compile_command = "gcc -fPIC -shared -O3 " + 
    prefix_code + func_name + " -o " +
    prefix_lib  + lib_name;

    std::cout << compile_command << std::endl;

    int compile_flag = std::system(compile_command.c_str());
    casadi_assert(compile_flag==0, "COMILATION FAILURE!");
    std::cout << "COMPILATION SUCCESS!" << std::endl;

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1);
    std::cout << "FUNCTION GENERATION SUCCESS DURING " << duration.count() << " seconds." << std::endl;

    return 0;
}