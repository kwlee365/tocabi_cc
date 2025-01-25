#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <filesystem>

using namespace casadi;

// https://groups.google.com/g/casadi-users/c/FNqBF6ilFgc?pli=1

std::string current_path = std::filesystem::current_path().parent_path().parent_path().string();
std::string prefix_code  = current_path + "/mpc_function/";   // The user should modify this variable your own directory.
std::string prefix_lib   = current_path + "/mpc_lib/";
std::string func_name    = "mpc_func.c";
std::string lib_name     = "lib_mpc_func.so";

int state_length = 12;  // [theta, p, w, pdot, g] in R{13}
int input_length = 12;  // [mL, fL, mR, fR] in R{12}
int N = 10;

int main(){
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // SYSTEM PARAMETER
    SX m = SX::sym("m");    
    SX g = SX::sym("g");    
    SX I = SX::sym("I", 3, 3);  
    SX I_inv = SX::inv(I);

    SX mu = SX::sym("mu");  // FRICTION COEFFICIENT

    SX dT = SX::sym("dT");  // MPC TIMESTEP

    SX rL_ref_horizon = SX::sym("rL_ref_horizon", 3, N);    // POSITION VECTOR FROM COM TO CONTACT LOCATION
    SX rR_ref_horizon = SX::sym("rR_ref_horizon", 3, N);
    
    SX theta_ref_horizon   = SX::sym("theta_ref_horizon", 3, N);    // BODY ORIENTATION W.R.T. GLOBAL FRAME

    SX etaL_ref_horizon = SX::sym("etaL_ref_horizon", N);
    SX etaR_ref_horizon = SX::sym("etaR_ref_horizon", N);

    // STATE VARIABLE
    SX X     = SX::sym("X",     state_length * N);    
    SX X_ref = SX::sym("X_ref", state_length * N);    
    SX U     = SX::sym("U",     input_length * N);
    SX U_ref = SX::sym("U_ref", input_length * N);

    SX v = vertcat(X, U);

    SX x0 = SX::sym("x0", state_length);  // CURRENT ROBOT STATE

    // COST FUNCTION
    SX W_Q = SX::sym("W_Q", N * state_length);  // TRAJECTORY TRACKING
    SX W_R = SX::sym("W_R", N * input_length);  // REGULATION
    SX J = mtimes(transpose(X - X_ref), mtimes(diag(W_Q), X - X_ref)) + mtimes(transpose(U - U_ref), mtimes(diag(W_R), U - U_ref));
    SX J_v = jacobian(J, v);
    SX J_vv = hessian(J, v);

    // EQUALITY CONSTRAINT
    SX ceq1;
    SX x_k = x0;
    
    for(int i = 0; i < N; i++)
    {
        SX u_k = U(Slice(input_length * i, input_length * (i+1)));

        SX rL = rL_ref_horizon(Slice(), i);
        SX rR = rR_ref_horizon(Slice(), i);

        SX theta = theta_ref_horizon(Slice(), i); 

        SX etaL = etaL_ref_horizon(i);
        SX etaR = etaR_ref_horizon(i);

        SX T = SX::zeros(3, 3);
        SX roll = theta(0); SX pitch = theta(1); SX yaw = theta(2);

        T(0, 0) = cos(pitch) * cos(yaw);
        T(0, 1) =-sin(yaw);
        T(0, 2) = 0.0;
        T(1, 0) = cos(pitch) * sin(yaw);
        T(1, 1) = cos(yaw);
        T(1, 2) = 0.0;
        T(2, 0) =-sin(pitch);
        T(2, 1) = 0.0;
        T(2, 2) = 1.0; 

        SX T_inv = SX::inv(T);

        // CONTINUOUS SYSTEM (LINEAR TIME-VARYING)
        SX A = SX::zeros(state_length, state_length);
        SX B = SX::zeros(state_length, input_length);
        SX d = SX::zeros(state_length, 1);

        A(Slice(0, 3), Slice(6, 9)) = T_inv;  
        A(Slice(3, 6), Slice(9,12)) = SX::eye(3);

        B(Slice(6, 9), Slice(0, 3)) = etaL * I_inv;
        B(Slice(6, 9), Slice(3, 6)) = etaL * mtimes(I_inv, skew(rL));
        B(Slice(6, 9), Slice(6, 9)) = etaR * I_inv;
        B(Slice(6, 9), Slice(9,12)) = etaR * mtimes(I_inv, skew(rR));

        B(Slice(9,12), Slice(3, 6)) = etaL * SX::eye(3) / m;
        B(Slice(9,12), Slice(9,12)) = etaR * SX::eye(3) / m;

        d(11) = -g;

        // DISCRETE SYSTEM (LINEAR TIME-VARYING)
        // SX Ad = SX::zeros(state_length, state_length);
        // SX Bd = SX::zeros(state_length, input_length);
        // SX dd = SX::zeros(state_length, 1);

        // Ad = (SX::eye(state_length) + A * dT);
        // Bd = B * dT;
        // dd = d * dT;
        
        // SX x_k_next = mtimes(Ad, x_k) + mtimes(Bd, u_k) + dd;

        SX k1 = mtimes(A, x_k                ) + mtimes(B, u_k) + d;
        SX k2 = mtimes(A, x_k + 0.5 * dT * k1) + mtimes(B, u_k) + d;
        SX k3 = mtimes(A, x_k + 0.5 * dT * k2) + mtimes(B, u_k) + d;
        SX k4 = mtimes(A, x_k +       dT * k3) + mtimes(B, u_k) + d;

        SX x_k_next = x_k + (dT / 6) * (k1 + 2 * k2 + 2 * k3 + k4);

        SX ceq1_sub = X(Slice(state_length * i, state_length * (i+1))) - x_k_next;

        ceq1 = vertcat(ceq1, ceq1_sub);

        x_k = x_k_next;
    }

    casadi::SX ceq1_v = jacobian(ceq1, v);

    // FRICTION CONE CONSTRAINTS
    SX cineq1_max; SX cineq1_min;
    SX cineq2_max; SX cineq2_min;
    SX cineq3_max; SX cineq3_min;
    SX cineq4_max; SX cineq4_min;
    SX cineq5_max; SX cineq5_min;

    SX f_z_max = SX::sym("f_z_max");
    SX f_z_min = SX::sym("f_z_min");

    SX footX = SX::sym("footX");    // FOOT WIDTH
    SX footY = SX::sym("footY");    // FOOT HEIGHT

    for (int i = 0; i < N; i++)
    {
        SX mL_x = U(input_length * i + 0);
        SX mL_y = U(input_length * i + 1);
        SX mL_z = U(input_length * i + 2);
        SX fL_x = U(input_length * i + 3);
        SX fL_y = U(input_length * i + 4);
        SX fL_z = U(input_length * i + 5);

        SX mR_x = U(input_length * i + 6);
        SX mR_y = U(input_length * i + 7);
        SX mR_z = U(input_length * i + 8);
        SX fR_x = U(input_length * i + 9);
        SX fR_y = U(input_length * i + 10);
        SX fR_z = U(input_length * i + 11);

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
    }

    casadi::SX cineq1_max_v = jacobian(cineq1_max, v);
    casadi::SX cineq1_min_v = jacobian(cineq1_min, v);
    casadi::SX cineq2_max_v = jacobian(cineq2_max, v);
    casadi::SX cineq2_min_v = jacobian(cineq2_min, v);
    casadi::SX cineq3_max_v = jacobian(cineq3_max, v);
    casadi::SX cineq3_min_v = jacobian(cineq3_min, v);
    casadi::SX cineq4_max_v = jacobian(cineq4_max, v);
    casadi::SX cineq4_min_v = jacobian(cineq4_min, v);
    casadi::SX cineq5_max_v = jacobian(cineq5_max, v);
    casadi::SX cineq5_min_v = jacobian(cineq5_min, v);

    // Generate CasADi functions
    Function J_v_func("J_v_func",   {X, U, X_ref, U_ref, W_Q, W_R}, {J_v});
    Function J_vv_func("J_vv_func", {X, U, X_ref, U_ref, W_Q, W_R}, {J_vv});

    Function ceq1_func("ceq1_func",     {x0, X, U, m, g, I, dT, rL_ref_horizon, rR_ref_horizon, theta_ref_horizon, etaL_ref_horizon, etaR_ref_horizon}, {ceq1});
    Function ceq1_v_func("ceq1_v_func", {x0, X, U, m, g, I, dT, rL_ref_horizon, rR_ref_horizon, theta_ref_horizon, etaL_ref_horizon, etaR_ref_horizon}, {ceq1_v});

    Function cineq1_max_func("cineq1_max_func", {U, f_z_max}, {cineq1_max});
    Function cineq1_min_func("cineq1_min_func", {U, f_z_min}, {cineq1_min});
    Function cineq2_max_func("cineq2_max_func", {U, mu}, {cineq2_max});
    Function cineq2_min_func("cineq2_min_func", {U, mu}, {cineq2_min});
    Function cineq3_max_func("cineq3_max_func", {U, mu}, {cineq3_max});
    Function cineq3_min_func("cineq3_min_func", {U, mu}, {cineq3_min});
    Function cineq4_max_func("cineq4_max_func", {U, footY}, {cineq4_max});
    Function cineq4_min_func("cineq4_min_func", {U, footY}, {cineq4_min});
    Function cineq5_max_func("cineq5_max_func", {U, footX}, {cineq5_max});
    Function cineq5_min_func("cineq5_min_func", {U, footX}, {cineq5_min});

    Function cineq1_max_v_func("cineq1_max_v_func", {U, f_z_max}, {cineq1_max_v});
    Function cineq1_min_v_func("cineq1_min_v_func", {U, f_z_min}, {cineq1_min_v});
    Function cineq2_max_v_func("cineq2_max_v_func", {U, mu}, {cineq2_max_v});
    Function cineq2_min_v_func("cineq2_min_v_func", {U, mu}, {cineq2_min_v});
    Function cineq3_max_v_func("cineq3_max_v_func", {U, mu}, {cineq3_max_v});
    Function cineq3_min_v_func("cineq3_min_v_func", {U, mu}, {cineq3_min_v});
    Function cineq4_max_v_func("cineq4_max_v_func", {U, footY}, {cineq4_max_v});
    Function cineq4_min_v_func("cineq4_min_v_func", {U, footY}, {cineq4_min_v});
    Function cineq5_max_v_func("cineq5_max_v_func", {U, footX}, {cineq5_max_v});
    Function cineq5_min_v_func("cineq5_min_v_func", {U, footX}, {cineq5_min_v});

    /////////////////////////
    // Function Generation //
    std::cout << "CASADI FUNCTION GENERATION START!!!" << std::endl;
    casadi::Dict opts = casadi::Dict();
    opts["cpp"] = false; opts["with_header"] = true;    
    casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(func_name, opts);
    myCodeGen.add(J_v_func);
    myCodeGen.add(J_vv_func);

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