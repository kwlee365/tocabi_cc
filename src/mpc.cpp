#include "mpc.h"

using namespace casadi;

MPC::MPC(RobotData &rd, double initMpcFreq, double initN) : rd_(rd), mpc_freq(initMpcFreq), N(initN)
{
    std::cout << "MPC FREQ: " << mpc_freq << "Hz" << std::endl;
    std::cout << "MPC HORIZON: " << N / mpc_freq << "s" << std::endl;
    std::cout << "MPC CLASS IS SUCCESSFULLY CONSTRUCTED" << std::endl;
}

void MPC::parameterSRBD()
{
    if(is_param_init_ == true)
    {
        mass_ = rd_.link_[COM_id].mass;
        inertia_ = rd_.link_[COM_id].inertia;   // global

        is_param_init_ = false;
    }
}

void MPC::modelSRBD()
{
    
    parameterSRBD();

    // SYSTEM PARAMETER
    SX m = SX::sym("m");    
    SX g = SX::sym("g");    
    SX I = SX::sym("I", 3, 3);  
    SX mu = SX::sym("mu");  // friction coefficient

    SX dT = SX::sym("dT");

    SX p_lf_ref_horizon = SX::sym("p_lf_ref_horizon", 3, N);      // position vector from COM to Stance foot
    SX p_rf_ref_horizon = SX::sym("p_rf_ref_horizon", 3, N);      // position vector from COM to Stance foot
    
    SX theta_ref_horizon = SX::sym("r", 3, N); 
    SX p_c_ref_horizon = SX::sym("r", 3, N); 
    SX w_ref_horizon = SX::sym("r", 3, N); 
    SX p_c_dot_ref_horizon = SX::sym("r", 3, N); 

    // STATE VARIABLE
    SX X = SX::sym("X", state_length * N);    
    SX X_ref = SX::sym("X_ref", state_length * N);    
    SX U = SX::sym("U", input_length * N);
    // SX U_ref = SX::sym("U_ref", input_length * N);    
    SX U_ref = SX::zeros(input_length * N);
    SX v = vertcat(X, U);

    SX x = SX::sym("x", state_length);  // current robot state

    // COST FUNCTION
    SX W_Q = SX::sym("W_Q", N * state_length);  // Trajectory Tracking
    SX W_R = SX::sym("W_R", N * input_length);  // Regulation
    SX J = mtimes(transpose(X - X_ref), mtimes(diag(W_Q), X - X_ref)) + mtimes(transpose(U - U_ref), mtimes(diag(W_R), U - U_ref));
    SX J_v = jacobian(J, v);
    SX J_vv = hessian(J, v);

    // STATE SPACE EQUATION
    std::vector<SX> A_k;
    std::vector<SX> B_k;
    std::vector<SX> d_k;

    // EQUALITY CONSTRAINT (SRBD)
    SX ceq1;
    SX x_k = x;
    for(int i = 0; i < N; i++)
    {
        SX ceq1_sub = SX::zeros(12, 1);

        SX x_k_next = SX::zeros(12, 1); // x_(k+1) = A_k * x_k + B_k * u_k
        SX u_k = SX::zeros(12, 1);  // u_k

        u_k = U(Slice(state_length * i + 0, state_length * i + 12));

        SX p_lf = p_lf_ref_horizon(Slice(), i);
        SX p_rf = p_rf_ref_horizon(Slice(), i);

        SX theta   = theta_ref_horizon(Slice(), i); // Build system matrix using the reference values.
        SX p_c     = p_c_ref_horizon(Slice(), i);
        SX w       = w_ref_horizon(Slice(), i);
        SX p_c_dot = p_c_dot_ref_horizon(Slice(), i);

        SX T = SX::zeros(3, 3);
        SX roll = theta(0); SX pitch= theta(1); SX yaw  = theta(2);

        T(0, 0) = cos(pitch) * cos(yaw);
        T(0, 1) = -sin(yaw);
        T(0, 2) = 0.0;
        T(1, 0) = cos(pitch) * sin(yaw);
        T(1, 1) = cos(yaw);
        T(1, 2) = 0.0;
        T(2, 0) = -sin(pitch);
        T(2, 1) = 0.0;
        T(2, 2) = 1.0; 

        SX T_inv = SX::inv(T);

        // CONTINUOUS SYSTEM
        SX A = SX::zeros(state_length, state_length);
        SX B = SX::zeros(state_length, input_length);
        SX d = SX::zeros(state_length, 1);

        A(Slice(0, 3), Slice(6, 9))  = T_inv;  
        A(Slice(3, 6), Slice(9,12)) = SX::eye(3);

        B(Slice(6, 9), Slice(0, 3)) = I;
        B(Slice(6, 9), Slice(3, 6)) = I * skew(p_lf);
        B(Slice(6, 9), Slice(6, 9)) = I;
        B(Slice(6, 9), Slice(9,12)) = I * skew(p_rf);

        B(Slice(9,12), Slice(3, 6)) = SX::eye(3) / m;
        B(Slice(9,12), Slice(9,12)) = SX::eye(3) / m;

        d(11) = -g;
    
        // DISCRETE SYSTEM
        SX Ad = SX::zeros(state_length, state_length);
        SX Bd = SX::zeros(state_length, input_length);
        SX dd = SX::zeros(state_length, 1);

        Ad = (SX::eye(12) + A * dT);
        Bd = B * dT;
        dd = d * dT;

        x_k_next = Ad * x_k + Bd * u_k + dd;

        ceq1_sub = X(Slice(state_length * i + 0, state_length * i + 12)) - x_k_next;

        ceq1 = vertcat(ceq1, ceq1_sub);

        x_k = x_k_next;
    }

    casadi::SX ceq1_v = jacobian(ceq1, v);

    // FRICTION CONE CONSTRAINTS
    SX cineq1;
    SX foot_X;
    SX foot_Y;
    for(int i = 0; i < N; i++)
    {
        SX cineq1_max_mL_sub = SX::zeros(2, 1);
        SX cineq1_min_mL_sub = SX::zeros(2, 1);
        SX cineq1_max_fL_sub = SX::zeros(2, 1);
        SX cineq1_min_fL_sub = SX::zeros(2, 1);

        SX cineq1_max_mR_sub = SX::zeros(2, 1);
        SX cineq1_min_mR_sub = SX::zeros(2, 1);
        SX cineq1_max_fR_sub = SX::zeros(2, 1);
        SX cineq1_min_fR_sub = SX::zeros(2, 1);

        SX mL = SX::zeros(3, 1);
        SX fL = SX::zeros(3, 1);
        SX mR = SX::zeros(3, 1);
        SX fR = SX::zeros(3, 1);

        mL = U(Slice(state_length * i + 0, state_length * i + 3));
        fL = U(Slice(state_length * i + 3, state_length * i + 6));
        mR = U(Slice(state_length * i + 6, state_length * i + 9));
        fR = U(Slice(state_length * i + 9, state_length * i +12));

        cineq1_max_mL_sub(0) = mL(0) - foot_Y * fL(2);    // tau_x <= Y * fz
        cineq1_max_mL_sub(1) = mL(1) - foot_X * fL(2);    // tau_y <= X * fz
        cineq1_min_mL_sub(0) =-mL(0) + foot_Y * fL(2);    //-tau_x <= Y * fz
        cineq1_min_mL_sub(1) =-mL(1) + foot_X * fL(2);    //-tau_y <= X * fz
        cineq1_max_fL_sub(0) = fL(0) - mu * fL(2);        // fx <= mu * fz
        cineq1_max_fL_sub(1) = fL(1) - mu * fL(2);        // fy <= mu * fz
        cineq1_min_fL_sub(0) =-fL(0) + mu * fL(2) ;       //-fx <= mu * fz
        cineq1_min_fL_sub(1) =-fL(1) + mu * fL(2) ;       //-fy <= mu * fz

        cineq1_max_mR_sub(0) = mR(0) - foot_Y * fR(2);    // tau_x <= Y * fz
        cineq1_max_mR_sub(1) = mR(1) - foot_X * fR(2);    // tau_y <= X * fz
        cineq1_min_mR_sub(0) =-mR(0) + foot_Y * fR(2);    //-tau_x <= Y * fz
        cineq1_min_mR_sub(1) =-mR(1) + foot_X * fR(2);    //-tau_y <= X * fz
        cineq1_max_fR_sub(0) = fR(0) - mu * fR(2);        // fx <= mu * fz
        cineq1_max_fR_sub(1) = fR(1) - mu * fR(2);        // fy <= mu * fz
        cineq1_min_fR_sub(0) =-fR(0) + mu * fR(2) ;       //-fx <= mu * fz
        cineq1_min_fR_sub(1) =-fR(1) + mu * fR(2) ;       //-fy <= mu * fz

        cineq1 = vertcat(cineq1, cineq1_max_mL_sub, cineq1_min_mL_sub);
        cineq1 = vertcat(cineq1, cineq1_max_fL_sub, cineq1_min_fL_sub);
        cineq1 = vertcat(cineq1, cineq1_max_mR_sub, cineq1_min_mR_sub);
        cineq1 = vertcat(cineq1, cineq1_max_fR_sub, cineq1_min_fR_sub);
    }

    casadi::SX cineq1_v = jacobian(cineq1, v);

    // BIG M CONSTRAINTS
    SX cineq2;
    SX eta_lf = SX::sym("eta_lf", 12);  // eta = 1 if foot is on swing.
                                        // eta = 0 else (support)
    SX eta_rf = SX::sym("eta_rf", 12);
    SX bigM = SX::sym("bigM");

    for (int i = 0; i < N; i++)
    {
        SX cineq2_max_uL_sub = SX::zeros(3, 1);
        SX cineq2_min_uL_sub = SX::zeros(3, 1);

        SX cineq2_max_uR_sub = SX::zeros(3, 1);
        SX cineq2_min_uR_sub = SX::zeros(3, 1);

        SX uL = SX::zeros(6, 1);
        SX uR = SX::zeros(6, 1);

        uL = U(Slice(state_length * i + 0, state_length * i + 6));
        uR = U(Slice(state_length * i + 6, state_length * i + 12));

        cineq2_max_uL_sub = uL - bigM * (1 - eta_lf(i));
        cineq2_min_uL_sub =-uL - bigM * (1 - eta_lf(i));

        cineq2_max_uR_sub = uR - bigM * (1 - eta_rf(i));
        cineq2_min_uR_sub =-uR - bigM * (1 - eta_rf(i));

        cineq2 = vercat(cineq2, cineq2_max_uL_sub, cineq2_min_uL_sub);
        cineq2 = vercat(cineq2, cineq2_max_uR_sub, cineq2_min_uR_sub);
    }

    /*
    // TODO : MAX_MIN 나누어서 제약 조건 다시 작성.
    // TODO : A 구성시에 ETA 반영?
    // TODO : cc랑 연동
    */
}

