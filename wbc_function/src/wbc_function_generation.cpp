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

const int nq = 40;
const int nv = 39;
const int na = 33;

int main(){
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::pair<int, int> dims;

    //--- System Parameter
    SX H = SX::sym("H", nv, nv);
    SX G = SX::sym("G", nv);
    SX J_c = SX::sym("J_c", 12, nv);
    SX J_i = SX::sym("J_i", 3, nv);
    SX mu = SX::sym("mu");

    SX torque = SX::sym("torque", na);
    SX lambda = SX::sym("lambda", 12);

    SX q = SX::sym("q", na);
    SX qdot = SX::sym("qdot", na);
    SX alpha1 = SX::sym("alpha1");
    SX alpha2 = SX::sym("alpha2");

    SX torque_lim  = SX::sym("torque_lim", na);
    SX q_pos_l_lim = SX::sym("q_pos_l_lim", na);
    SX q_pos_h_lim = SX::sym("q_pos_h_lim", na);

    SX qddot_des = SX::sym("qddot_des", nv);
    SX torque_prev = SX::sym("torque_prev", na);

    SX S_T = SX::zeros(nv, na);
    for (int i = nv - na; i < nv; ++i) S_T(i, i - (nv - na)) = 1.0;
    SX A = horzcat(S_T, transpose(J_c));
    SX F = vertcat(torque, lambda);
    SX bdot = mtimes(H, qddot_des) + G;


    

    
    //--- COST FUNCTION
    SX W_Q = SX::sym("W_Q", nv);
    SX W_torque = SX::sym("W_torque", na);
    SX W_lambda = SX::sym("W_lambda", 12);
    SX W_torque_prev = SX::sym("W_torque_prev", na);

    SX J1 = mtimes(transpose(mtimes(A, F) - bdot), mtimes(diag(W_Q), (mtimes(A, F) - bdot)));
    SX J2 = mtimes(transpose(torque), mtimes(diag(W_torque), torque));
    SX J3 = mtimes(transpose(lambda), mtimes(diag(W_lambda), lambda));
    SX J4 = mtimes(transpose(torque - torque_prev), mtimes(diag(W_torque_prev), torque - torque_prev));

    SX J = J1 + J2 + J3 + J4;
    SX J_v = jacobian(J, F);
    SX J_vv = hessian(J, F);





    // //--- CONTACT CONSTRAINTS
    SX ceq0 = mtimes(J_c, solve(H, mtimes(A, F) - G))- mtimes(J_c, qddot_des);
    SX ceq0_v = jacobian(ceq0, F);

    //--- VIRTUAL JOINTS CONSTRAINTS
    SX ceq1_full = mtimes(H, qddot_des) + G - mtimes(transpose(J_c), lambda);
    SX ceq1 = ceq1_full(Slice(0, 6)); 
    SX ceq1_v = jacobian(ceq1, F);


    //--- FRICTION CONE CONSTRAINTS
    SX cineq1_max; SX cineq1_min;
    SX cineq2_max; SX cineq2_min;
    SX cineq3_max; SX cineq3_min;
    SX cineq4_max; SX cineq4_min;
    SX cineq5_max; SX cineq5_min;

    SX f_z_max = SX::sym("f_z_max");
    SX f_z_min = SX::sym("f_z_min");

    SX footX = SX::sym("footX");    // FOOT WIDTH
    SX footY = SX::sym("footY");    // FOOT HEIGHT

    SX fL_x = lambda(0);
    SX fL_y = lambda(1);
    SX fL_z = lambda(2);
    SX mL_x = lambda(3);
    SX mL_y = lambda(4);
    SX mL_z = lambda(5);

    SX fR_x = lambda(6);
    SX fR_y = lambda(7);
    SX fR_z = lambda(8);
    SX mR_x = lambda(9);
    SX mR_y = lambda(10);
    SX mR_z = lambda(11);

    // UNILATERAL CONTACT CONDITION
    SX cineq1_max_l_sub = fL_z - f_z_max;
    SX cineq1_max_r_sub = fR_z - f_z_max;
    SX cineq1_min_l_sub =-fL_z + f_z_min; 
    SX cineq1_min_r_sub =-fR_z + f_z_min;
    cineq1_max = vertcat(cineq1_max, cineq1_max_l_sub, cineq1_max_r_sub);
    cineq1_min = vertcat(cineq1_min, cineq1_min_l_sub, cineq1_min_r_sub);

    // NO SLIP CONDITION (HORIZONTAL FORCE, X)
    SX cineq2_max_l_sub = fL_x - mu * fL_z; 
    SX cineq2_max_r_sub = fR_x - mu * fR_z;
    SX cineq2_min_l_sub =-fL_x - mu * fL_z; 
    SX cineq2_min_r_sub =-fR_x - mu * fR_z;
    cineq2_max = vertcat(cineq2_max, cineq2_max_l_sub, cineq2_max_r_sub);
    cineq2_min = vertcat(cineq2_min, cineq2_min_l_sub, cineq2_min_r_sub);

    // NO SLIP CONDITION (HORIZONTAL FORCE, Y)
    SX cineq3_max_l_sub = fL_y - mu * fL_z; 
    SX cineq3_max_r_sub = fR_y - mu * fR_z;
    SX cineq3_min_l_sub =-fL_y - mu * fL_z; 
    SX cineq3_min_r_sub =-fR_y - mu * fR_z;
    cineq3_max = vertcat(cineq3_max, cineq3_max_l_sub, cineq3_max_r_sub);
    cineq3_min = vertcat(cineq3_min, cineq3_min_l_sub, cineq3_min_r_sub);

    // NO TIPPING CONDITION (HORIZONTAL MOMENT, X)
    SX cineq4_max_l_sub = mL_x - footY * fL_z; 
    SX cineq4_max_r_sub = mR_x - footY * fR_z;
    SX cineq4_min_l_sub =-mL_x - footY * fL_z; 
    SX cineq4_min_r_sub =-mR_x - footY * fR_z;
    cineq4_max = vertcat(cineq4_max, cineq4_max_l_sub, cineq4_max_r_sub);
    cineq4_min = vertcat(cineq4_min, cineq4_min_l_sub, cineq4_min_r_sub);

    // NO TIPPING CONDITION (HORIZONTAL MOMENT, Y)
    SX cineq5_max_l_sub = mL_y - footX * fL_z; 
    SX cineq5_max_r_sub = mR_y - footX * fR_z;
    SX cineq5_min_l_sub =-mL_y - footX * fL_z; 
    SX cineq5_min_r_sub =-mR_y - footX * fR_z;
    cineq5_max = vertcat(cineq5_max, cineq5_max_l_sub, cineq5_max_r_sub);
    cineq5_min = vertcat(cineq5_min, cineq5_min_l_sub, cineq5_min_r_sub);

    SX cineq1_max_v = jacobian(cineq1_max, F);
    SX cineq1_min_v = jacobian(cineq1_min, F);
    SX cineq2_max_v = jacobian(cineq2_max, F);
    SX cineq2_min_v = jacobian(cineq2_min, F);
    SX cineq3_max_v = jacobian(cineq3_max, F);
    SX cineq3_min_v = jacobian(cineq3_min, F);
    SX cineq4_max_v = jacobian(cineq4_max, F);
    SX cineq4_min_v = jacobian(cineq4_min, F);
    SX cineq5_max_v = jacobian(cineq5_max, F);
    SX cineq5_min_v = jacobian(cineq5_min, F);





    //--- TORQUE BOUNDARY CONSTRAINTS
    SX cineq6_max; SX cineq6_min;
    cineq6_max =  torque - torque_lim;
    cineq6_min = -torque - torque_lim;
    SX cineq6_max_v = jacobian(cineq6_max, F);
    SX cineq6_min_v = jacobian(cineq6_min, F);





    //--- JOINT POSITION BOUNDARY CONSTRAINTS
    SX qddot_actual = solve(H, mtimes(A, F) - G); // qddot = H⁻¹(AF - G)
    SX qddot_act_na = qddot_actual(Slice(nv - na, nv));  // Only actuated joints

    SX cineq7_max = qddot_act_na + (alpha1 + alpha2) * qdot + alpha1 * alpha2 * (q - q_pos_h_lim);
    SX cineq7_min =-qddot_act_na - (alpha1 + alpha2) * qdot - alpha1 * alpha2 * (q - q_pos_l_lim);
    SX cineq7_max_v = jacobian(cineq7_max, F);
    SX cineq7_min_v = jacobian(cineq7_min, F);

    //--- GENERATE CASADI FUNCTIONS
    Function J_v_func("J_v_func",
        {H, G, J_c, qdot, qddot_des, torque, torque_prev, lambda, W_Q, W_torque, W_lambda, W_torque_prev},
        {J_v}
    );

    Function J_vv_func("J_vv_func",
        {H, G, J_c, qdot, qddot_des, torque, torque_prev, lambda, W_Q, W_torque, W_lambda, W_torque_prev},
        {J_vv}
    );

    // --- Contact constraint
    Function ceq0_func("ceq0_func", {H, G, J_c, qddot_des, torque, lambda}, {ceq0});
    Function ceq0_v_func("ceq0_v_func", {H, G, J_c, qddot_des, torque, lambda}, {ceq0_v});

    Function ceq1_func("ceq1_func", {H, G, J_c, qddot_des, lambda}, {ceq1});
    Function ceq1_v_func("ceq1_v_func", {H, G, J_c, qddot_des, lambda}, {ceq1_v});

    // --- Friction cone constraints
    Function cineq1_max_func("cineq1_max_func", {lambda, f_z_max}, {cineq1_max});
    Function cineq1_min_func("cineq1_min_func", {lambda, f_z_min}, {cineq1_min});
    Function cineq2_max_func("cineq2_max_func", {lambda, mu}, {cineq2_max});
    Function cineq2_min_func("cineq2_min_func", {lambda, mu}, {cineq2_min});
    Function cineq3_max_func("cineq3_max_func", {lambda, mu}, {cineq3_max});
    Function cineq3_min_func("cineq3_min_func", {lambda, mu}, {cineq3_min});
    Function cineq4_max_func("cineq4_max_func", {lambda, footY}, {cineq4_max});
    Function cineq4_min_func("cineq4_min_func", {lambda, footY}, {cineq4_min});
    Function cineq5_max_func("cineq5_max_func", {lambda, footX}, {cineq5_max});
    Function cineq5_min_func("cineq5_min_func", {lambda, footX}, {cineq5_min});

    // --- Derivatives of constraints
    Function cineq1_max_v_func("cineq1_max_v_func", {lambda, f_z_max}, {cineq1_max_v});
    Function cineq1_min_v_func("cineq1_min_v_func", {lambda, f_z_min}, {cineq1_min_v});
    Function cineq2_max_v_func("cineq2_max_v_func", {lambda, mu}, {cineq2_max_v});
    Function cineq2_min_v_func("cineq2_min_v_func", {lambda, mu}, {cineq2_min_v});
    Function cineq3_max_v_func("cineq3_max_v_func", {lambda, mu}, {cineq3_max_v});
    Function cineq3_min_v_func("cineq3_min_v_func", {lambda, mu}, {cineq3_min_v});
    Function cineq4_max_v_func("cineq4_max_v_func", {lambda, footY}, {cineq4_max_v});
    Function cineq4_min_v_func("cineq4_min_v_func", {lambda, footY}, {cineq4_min_v});
    Function cineq5_max_v_func("cineq5_max_v_func", {lambda, footX}, {cineq5_max_v});
    Function cineq5_min_v_func("cineq5_min_v_func", {lambda, footX}, {cineq5_min_v});

    // --- Torque limits
    Function cineq6_max_func("cineq6_max_func", {torque, torque_lim}, {cineq6_max});
    Function cineq6_min_func("cineq6_min_func", {torque, torque_lim}, {cineq6_min});
    Function cineq6_max_v_func("cineq6_max_v_func", {torque, torque_lim}, {cineq6_max_v});
    Function cineq6_min_v_func("cineq6_min_v_func", {torque, torque_lim}, {cineq6_min_v});

    // --- Joint limits
    Function cineq7_max_func("cineq7_max_func", {H, G, J_c, torque, lambda, q, qdot, q_pos_h_lim, alpha1, alpha2}, {cineq7_max});
    Function cineq7_min_func("cineq7_min_func", {H, G, J_c, torque, lambda, q, qdot, q_pos_l_lim, alpha1, alpha2}, {cineq7_min});
    Function cineq7_max_v_func("cineq7_max_v_func", {H, G, J_c, torque, lambda, q, qdot, q_pos_h_lim, alpha1, alpha2}, {cineq7_max_v});
    Function cineq7_min_v_func("cineq7_min_v_func", {H, G, J_c, torque, lambda, q, qdot, q_pos_l_lim, alpha1, alpha2}, {cineq7_min_v});

    /////////////////////////
    // Function Generation //
    std::cout << "CASADI FUNCTION GENERATION START!!!" << std::endl;
    Dict opts = Dict();
    opts["cpp"] = false; opts["with_header"] = true;    
    CodeGenerator myCodeGen = CodeGenerator(func_name, opts);
    myCodeGen.add(J_v_func);
    myCodeGen.add(J_vv_func);

    myCodeGen.add(ceq0_func);
    myCodeGen.add(ceq0_v_func);

    myCodeGen.add(ceq1_func);
    myCodeGen.add(ceq1_v_func);

    myCodeGen.add(cineq1_max_func);
    myCodeGen.add(cineq1_min_func);
    myCodeGen.add(cineq2_max_func);
    myCodeGen.add(cineq2_min_func);
    myCodeGen.add(cineq3_max_func);
    myCodeGen.add(cineq3_min_func);
    myCodeGen.add(cineq4_max_func);
    myCodeGen.add(cineq4_min_func);
    myCodeGen.add(cineq5_max_func);
    myCodeGen.add(cineq5_min_func);
    myCodeGen.add(cineq6_max_func);
    myCodeGen.add(cineq6_min_func);
    myCodeGen.add(cineq7_max_func);
    myCodeGen.add(cineq7_min_func);

    myCodeGen.add(cineq1_max_v_func);
    myCodeGen.add(cineq1_min_v_func);
    myCodeGen.add(cineq2_max_v_func);
    myCodeGen.add(cineq2_min_v_func);
    myCodeGen.add(cineq3_max_v_func);
    myCodeGen.add(cineq3_min_v_func);
    myCodeGen.add(cineq4_max_v_func);
    myCodeGen.add(cineq4_min_v_func);
    myCodeGen.add(cineq5_max_v_func);
    myCodeGen.add(cineq5_min_v_func);
    myCodeGen.add(cineq6_max_v_func);
    myCodeGen.add(cineq6_min_v_func);
    myCodeGen.add(cineq7_max_v_func);
    myCodeGen.add(cineq7_min_v_func);

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