function u_opt = mpc_controller_2uav(x_current)

    persistent U_prev initialized mpc_call_count input_states_log output_controls_log solve_times_log
    
    % Start timing for this MPC cycle
    tic_start = tic;
    
    % Track function call count
    if isempty(mpc_call_count)
        mpc_call_count = 0;
    end
    mpc_call_count = mpc_call_count + 1;
    
    % Initialize logging arrays
    if isempty(input_states_log)
        input_states_log = [];
        output_controls_log = [];
        solve_times_log = [];
    end
    
    %% Physical parameters
    m   = 1.5;               
    mP  = 1;                
    g   = 9.80665;          
    Ixx = 0.022; Iyy = 0.022; Izz = 0.04; 
    
    % Tether parameters
    L   = 1.6;                
    k   = 80;                 
    b   = 2.0;                
    
    %% MPC settings
    dt = 0.05;       
    N  = 10;         
    % nx = 30; 
    nu = 8;                   
               
    w_u  = diag([0.03 1.015 1.015 1.015]);                 
    
    %% Constraints
    phi_max   = deg2rad(45);
    theta_max = deg2rad(45);
    
    T_min = 0.0;   
    T_max = 34.0;  
    
    tau_phi_min   = -0.3; tau_phi_max   = 0.3;
    tau_theta_min = -0.3; tau_theta_max = 0.3;
    tau_psi_min   = -0.2; tau_psi_max   = 0.2;
    
    %% Initialize on first call
    if isempty(initialized)
        % Warm start (hover thrust for each UAV)
        U_prev = repmat([(mP*0.5+m)*g; 0; 0; 0; (mP*0.5+m)*g; 0; 0; 0], N, 1);
        initialized = true;
    end
    
    %% Helper functions
    f_step = @(x,u) step_dynamics_2uav(x,u,Ixx,Iyy,Izz,dt,m,mP,L,k,b,g);
    rollout = @(x0,U) simulate_rollout_2uav(x0,U,N,nu,f_step);
    
    %% Build bounds
    lb_u = [T_min; tau_phi_min; tau_theta_min; tau_psi_min; T_min; tau_phi_min; tau_theta_min; tau_psi_min];
    ub_u = [T_max; tau_phi_max; tau_theta_max; tau_psi_max; T_max; tau_phi_max; tau_theta_max; tau_psi_max];
    lbU = repmat(lb_u, N, 1);
    ubU = repmat(ub_u, N, 1);
    
    %% fmincon options
    opts = optimoptions('fmincon','Algorithm','sqp',...
        'Display','none','MaxIterations',50,'OptimalityTolerance',1e-1,...
        'StepTolerance',1e-2,'ConstraintTolerance',1e-2);
    
    %% Define cost function
    obj = @(U) stage_cost_2uav(x_current,U,N,nu,rollout,w_u);
    
    %% Define constraints
    nonlcon = @(U) nl_constr_angles_2uav(x_current,U,N,nu,rollout,phi_max,theta_max);
    
    %% Start timing for optimization solve
    solve_start = tic;
    
    %% Solve NMPC
    [Uopt,~,exitflag] = fmincon(obj, U_prev, [],[],[],[], lbU, ubU, nonlcon, opts);
    
    %% Record solve time
    solve_time = toc(solve_start);
    
    if exitflag <= 0
        % Use previous control if optimization fails
        Uopt = U_prev;
    end
    
    %% Extract first control action
    u_opt = Uopt(1:nu);
    
    %% Log input states and output controls
    input_states_log = [input_states_log, x_current];
    output_controls_log = [output_controls_log, u_opt];
    solve_times_log = [solve_times_log, solve_time];
    
    %% Update warm start for next iteration
    U_prev = [Uopt(nu+1:end); Uopt(end-nu+1:end)];
    
    %% Record total cycle time
    total_cycle_time = toc(tic_start);
    
    % Export call count and logged data to workspace when simulation runs
    assignin('base', 'mpc_controller_call_count', mpc_call_count);
    assignin('base', 'mpc_input_states_log', input_states_log);
    assignin('base', 'mpc_output_controls_log', output_controls_log);
    assignin('base', 'mpc_solve_times_log', solve_times_log);
end

%% Cost function
function [J,Xseq] = stage_cost_2uav(x0,U,N,nu,rollout,w_u)
    Xseq = rollout(x0,U);
    J = 0;
    
    dis_between_uav = 1.2;  
    
    for k=1:N
        xk = Xseq(:,k);
        uk = U((k-1)*nu+1:k*nu);
        
        % Extract UAV1 states (indices 1-12)
        N1 = xk(1); vN1 = xk(2); E1 = xk(3); vE1 = xk(4); vD1 = xk(6);
        
        % Extract UAV2 states (indices 13-24)
        N2 = xk(13); vN2 = xk(14); E2 = xk(15); vE2 = xk(16); vD2 = xk(18);
        
        Np = xk(25); vNp = xk(26); Ep = xk(27); vEp = xk(28); vDp = xk(30);
        
        % Split controls 
        u1 = uk(1:4);
        u2 = uk(5:8);
        
        % Kill payload oscillations (highest priority)
        J = J + 50.0 * (vNp^2 + vEp^2 + vDp^2);  

        payload_center_N = (N1 + N2) / 2;  
        payload_center_E = (E1 + E2) / 2;  

        J = J + 15.0 * ((Np - payload_center_N)^2 + (Ep - payload_center_E)^2);
        
        % Payload acceleration penalty
        if k > 1
            xk_prev = Xseq(:,k-1);
            vNp_prev = xk_prev(26); vEp_prev = xk_prev(28); vDp_prev = xk_prev(30);
            aNp = (vNp - vNp_prev) / 0.05;  % dt = 0.05
            aEp = (vEp - vEp_prev) / 0.05;
            aDp = (vDp - vDp_prev) / 0.05;

            J = J + 5.0 * (aNp^2 + aEp^2 + aDp^2);
        end
        
        J = J + 2 * ((vN1 - vN2)^2 + (vE1 - vE2)^2 + (vD1 - vD2)^2);
        
        J = J + 1 * (( (N1 - N2)^2 - dis_between_uav^2 )^2);

        J = J + 1 * (( (E1 - E2)^2 - dis_between_uav^2 )^2);

        J = J + u1'*w_u*u1 + u2'*w_u*u2;
    end
    
    % Terminal cost
    xT = Xseq(:,N+1);
    vNpT = xT(26); vEpT = xT(28); vDpT = xT(30);
    
    payload_vel_final = sqrt(vNpT^2 + vEpT^2 + vDpT^2);
    J = J + 100.0 * payload_vel_final^2;

    N1T = xT(1); N2T = xT(13); E1T = xT(3); E2T = xT(15);
    NpT = xT(25); EpT = xT(27);
    payload_center_NT = (N1T + N2T) / 2;
    payload_center_ET = (E1T + E2T) / 2;

    J = J + 30.0 * ((NpT - payload_center_NT)^2 + (EpT - payload_center_ET)^2);
    
    vN1T = xT(2); vE1T = xT(4); vD1T = xT(6);
    vN2T = xT(14); vE2T = xT(16); vD2T = xT(18);

    J = J + 35.0 * (vN1T^2 + vE1T^2 + vD1T^2 + vN2T^2 + vE2T^2 + vD2T^2);
end

%% Nonlinear constraints
function [c,ceq] = nl_constr_angles_2uav(x0,U,~,~,rollout,phi_max,theta_max)
    Xseq = rollout(x0,U);
    % UAV1 angles
    phi1   = Xseq(7,:);  theta1 = Xseq(9,:);
    % UAV2 angles
    phi2   = Xseq(19,:); theta2 = Xseq(21,:);
    % Inequalities c(x) <= 0 for both UAVs
    c = [ abs(phi1)-phi_max, abs(theta1)-theta_max, abs(phi2)-phi_max, abs(theta2)-theta_max ]';
    c = c(:);
    ceq = [];
end

%% Rollout simulation 
function [Xseq] = simulate_rollout_2uav(x0,U,N,nu,f_step)
    nx = numel(x0);
    Xseq = zeros(nx, N+1);
    Xseq(:,1) = x0;
    xk = x0;
    for k=1:N
        uk = U((k-1)*nu+1 : k*nu);
        xk = f_step(xk, uk);
        Xseq(:,k+1) = xk;
    end
end

%% Discrete dynamics (for MPC prediction)
function xnext = step_dynamics_2uav(x,u,Ixx,Iyy,Izz,dt,m,mP,L,k,b,g)
    % Extract UAV1
    x1 = x(1:12);
    x2 = x(13:24);
    xp = x(25:30);

    % UAV1 state
    N1 = x1(1); vN1 = x1(2); E1 = x1(3); vE1 = x1(4); D1 = x1(5); vD1 = x1(6);
    phi1 = x1(7); p1 = x1(8); th1 = x1(9); q1 = x1(10); psi1 = x1(11); r1 = x1(12);

    % UAV2 state
    N2 = x2(1); vN2 = x2(2); E2 = x2(3); vE2 = x2(4); D2 = x2(5); vD2 = x2(6);
    phi2 = x2(7); p2 = x2(8); th2 = x2(9); q2 = x2(10); psi2 = x2(11); r2 = x2(12);

    % Payload state
    Np = xp(1); vNp = xp(2); Ep = xp(3); vEp = xp(4); Dp = xp(5); vDp = xp(6);

    % Controls
    u1 = u(1:4); u2 = u(5:8);
    T1 = u1(1); tau_phi1 = u1(2); tau_theta1 = u1(3); tau_psi1 = u1(4);
    T2 = u2(1); tau_phi2 = u2(2); tau_theta2 = u2(3); tau_psi2 = u2(4);

    % Positions and velocities
    p1_vec = [N1; E1; D1]; v1_vec = [vN1; vE1; vD1];
    p2_vec = [N2; E2; D2]; v2_vec = [vN2; vE2; vD2];
    pp_vec = [Np; Ep; Dp]; vp_vec = [vNp; vEp; vDp];

    % Tether forces from UAVs to payload (spring-damper)  
    r1p = pp_vec - p1_vec; d1 = norm(r1p);
    if (d1 - L) > 0
        u1t = r1p / d1;
        F1_spring = k*(d1 - L)*u1t;
        vel_rel1 = vp_vec - v1_vec;
        F1_damping = b * vel_rel1;
        F1 = F1_spring + F1_damping;
    else
        F1 = [0;0;0];
    end

    r2p = pp_vec - p2_vec; d2 = norm(r2p);
    if (d2 - L) > 0
        u2t = r2p / d2;
        F2_spring = k*(d2 - L)*u2t;
        vel_rel2 = vp_vec - v2_vec;
        F2_damping = b * vel_rel2;
        F2 = F2_spring + F2_damping;
    else
        F2 = [0;0;0];
    end

    % Translational accelerations (NED) 
    aN1 = - (T1/m)*( sin(phi1)*sin(psi1) + cos(phi1)*sin(th1)*cos(psi1) ) + F1(1)/m;
    aE1 = - (T1/m)*( cos(phi1)*sin(th1)*sin(psi1) - sin(phi1)*cos(psi1) ) + F1(2)/m;
    aD1 = - (T1/m)*( cos(phi1)*cos(th1) ) + g + F1(3)/m;

    aN2 = - (T2/m)*( sin(phi2)*sin(psi2) + cos(phi2)*sin(th2)*cos(psi2) ) + F2(1)/m;
    aE2 = - (T2/m)*( cos(phi2)*sin(th2)*sin(psi2) - sin(phi2)*cos(psi2) ) + F2(2)/m;
    aD2 = - (T2/m)*( cos(phi2)*cos(th2) ) + g + F2(3)/m;

    % Rotational dynamics
    p1_dot = (tau_phi1   - (Iyy - Izz)*q1*r1)/Ixx;
    q1_dot = (tau_theta1 - (Izz - Ixx)*p1*r1)/Iyy;
    r1_dot = (tau_psi1   - (Ixx - Iyy)*p1*q1)/Izz;

    p2_dot = (tau_phi2   - (Iyy - Izz)*q2*r2)/Ixx;
    q2_dot = (tau_theta2 - (Izz - Ixx)*p2*r2)/Iyy;
    r2_dot = (tau_psi2   - (Ixx - Iyy)*p2*q2)/Izz;

    p1 = p1 + p1_dot*dt;  q1 = q1 + q1_dot*dt;  r1 = r1 + r1_dot*dt;
    p2 = p2 + p2_dot*dt;  q2 = q2 + q2_dot*dt;  r2 = r2 + r2_dot*dt;

    % Euler angle integration
    cth1 = cos(th1);  if abs(cth1) < 1e-6, cth1 = sign(cth1)*1e-6; end
    Winv1 = [ 1,  sin(phi1)*tan(th1),  cos(phi1)*tan(th1);
              0,  cos(phi1),          -sin(phi1);
              0,  sin(phi1)/cth1,      cos(phi1)/cth1 ];
    eul1_dot = Winv1 * [p1; q1; r1];
    phi1 = phi1 + eul1_dot(1)*dt;
    th1  = th1  + eul1_dot(2)*dt;
    psi1 = psi1 + eul1_dot(3)*dt;

    cth2 = cos(th2);  if abs(cth2) < 1e-6, cth2 = sign(cth2)*1e-6; end
    Winv2 = [ 1,  sin(phi2)*tan(th2),  cos(phi2)*tan(th2);
              0,  cos(phi2),          -sin(phi2);
              0,  sin(phi2)/cth2,      cos(phi2)/cth2 ];
    eul2_dot = Winv2 * [p2; q2; r2];
    phi2 = phi2 + eul2_dot(1)*dt;
    th2  = th2  + eul2_dot(2)*dt;
    psi2 = psi2 + eul2_dot(3)*dt;

    % Integrate UAV translations
    vN1 = vN1 + aN1*dt;   N1 = N1 + vN1*dt;
    vE1 = vE1 + aE1*dt;   E1 = E1 + vE1*dt;
    vD1 = vD1 + aD1*dt;   D1 = D1 + vD1*dt;

    vN2 = vN2 + aN2*dt;   N2 = N2 + vN2*dt;
    vE2 = vE2 + aE2*dt;   E2 = E2 + vE2*dt;
    vD2 = vD2 + aD2*dt;   D2 = D2 + vD2*dt;

    % Payload dynamics 
    aNp = - (F1(1) + F2(1)) / mP;
    aEp = - (F1(2) + F2(2)) / mP;
    aDp = - (F1(3) + F2(3)) / mP + g;

    % Integrate payload states
    vNp = vNp + aNp*dt;  Np = Np + vNp*dt;
    vEp = vEp + aEp*dt;  Ep = Ep + vEp*dt;
    vDp = vDp + aDp*dt;  Dp = Dp + vDp*dt;

    % Pack next state
    x1_next = [N1; vN1; E1; vE1; D1; vD1; phi1; p1; th1; q1; psi1; r1];
    x2_next = [N2; vN2; E2; vE2; D2; vD2; phi2; p2; th2; q2; psi2; r2];
    xp_next = [Np; vNp; Ep; vEp; Dp; vDp];

    xnext = [x1_next; x2_next; xp_next];
end
