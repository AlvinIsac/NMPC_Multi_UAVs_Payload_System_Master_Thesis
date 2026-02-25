clear; clc; close all;

%% Physical parameters
m   = 1.5;               
mP  = 1;                
g   = 9.80665;          
Ixx = 0.022; Iyy = 0.022; Izz = 0.04; 

% Tether parameters (softer for stability)
L   = 1.6;                
k   = 80;                 
b   = 2.0;                

%% Discretization and MPC settings
dt = 0.05;       
N  = 10;         
MAX_STEPS = 100;  
nx = 30; nu = 8;                   
           
w_u  = diag([0.03 0.015 0.015 0.015]);                 

%% Constraints
phi_max   = deg2rad(45);
theta_max = deg2rad(45);
psi_max   = pi;                       

T_min = 0.0;   
T_max = 34.0;  

tau_phi_min   = -0.3; tau_phi_max   = 0.3;
tau_theta_min = -0.3; tau_theta_max = 0.3;
tau_psi_min   = -0.2; tau_psi_max   = 0.2;

% UAV1 initial conditions
pN1     = -0.8;             
vN1    = 0.0;               
pE1     = 0.0;              
vE1    = 0.0;               
pD1     = -7;              
vD1    = 0.0;              

phi1   = deg2rad(0);       
p1     = 0;                
theta1 = deg2rad(0);       
q1     = 0;                
psi1   = deg2rad(-90);     
r1     = 0;                

% --- Second UAV initial conditions
pN2    = 0.8;              
vN2   = 0.0;               
pE2    = 0.0;              
vE2   = 0.0;               
pD2    = -7;              
vD2   = 0.0;              

phi2   = deg2rad(0);       
p2     = 0;                
theta2 = deg2rad(0);       
q2     = 0;                
psi2   = deg2rad(-90);     
r2     = 0;                

% Payload initial conditions
start_Np    = (pN1 + pN2) / 2;   
start_vNp   = 0.0;               
start_Ep    = 1.3;               
start_vEp   = 0.0;               
start_Dp    = pD1 + L - 0.3;          
start_vDp   = 0.0;               

% === state vector - 30 huge !! ===
x = [ pN1; vN1; pE1; vE1; pD1; vD1; phi1; p1; theta1; q1; psi1; r1; ... 
    pN2; vN2; pE2; vE2; pD2; vD2; phi2; p2; theta2; q2; psi2; r2; ... 
    start_Np; start_vNp; start_Ep; start_vEp; start_Dp; start_vDp ]; 

%% Storage for logs
logX = zeros(nx, MAX_STEPS+1);
logU = zeros(nu, MAX_STEPS);
logX(:,1) = x;

% For visualization 
pred_traj = cell(MAX_STEPS,1);

%% fmincon options
opts = optimoptions('fmincon','Algorithm','sqp',...
    'Display','none','MaxIterations',100,'OptimalityTolerance',1e-2,...
    'StepTolerance',1e-3,'ConstraintTolerance',1e-2);

%% Helper functions -------------------------------------------------------

f_step = @(x,u) step_dynamics_2uav(x,u,Ixx,Iyy,Izz,dt,m,mP,L,k,b,g);

% Rollout over horizon given current x0 and control sequence U (vectorized)
rollout = @(x0,U) simulate_rollout_2uav(x0,U,N,nu,f_step);

% Cost function for stabilization and oscillation damping (no position tracking)
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

        %  dont stop the payload velocity move along with it
        J = J + 50.0 * (vNp^2 + vEp^2 + vDp^2);  

        payload_center_N = (N1 + N2) / 2;  
        payload_center_E = (E1 + E2) / 2;  

        %  this makes the uav to move in oscillation direction
        J = J + 150.0 * ((Np - payload_center_N)^2 + (Ep - payload_center_E)^2);
        

        %  this is the magic boy !!
        if k > 1
            xk_prev = Xseq(:,k-1);
            vNp_prev = xk_prev(26); vEp_prev = xk_prev(28); vDp_prev = xk_prev(30);
            aNp = (vNp - vNp_prev) / 0.05;  % dt = 0.05
            aEp = (vEp - vEp_prev) / 0.05;
            aDp = (vDp - vDp_prev) / 0.05;

            J = J + 5.0 * (aNp^2 + aEp^2 + aDp^2);  % Penalty on payload acceleration
        end
        
        J = J + 2 * ((vN1 - vN2)^2 + (vE1 - vE2)^2 + (vD1 - vD2)^2);
        
        J = J + 1 * (( (N1 - N2)^2 - dis_between_uav^2 )^2);

        J = J + 1 * (( (E1 - E2)^2 - dis_between_uav^2 )^2);

        J = J + u1'*w_u*u1 + u2'*w_u*u2;
    end
    
    % Terminal cost see the magic of this comment and see it

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
    
    % Xseq = Xseq;
end

% Nonlinear constraints: angle limits for both UAVs
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

% Gives next states with Input
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
    if d1 > 1e-6
        u1t = r1p / d1;
        F1_spring = k*(d1 - L)*u1t;
        vel_rel1 = vp_vec - v1_vec;
        F1_damping = b * dot(vel_rel1, u1t) * u1t;
        F1 = F1_spring + F1_damping; % force on payload from UAV1
    else
        F1 = [0;0;0];
    end

    r2p = pp_vec - p2_vec; d2 = norm(r2p);
    if d2 > 1e-6
        u2t = r2p / d2;
        F2_spring = k*(d2 - L)*u2t;
        vel_rel2 = vp_vec - v2_vec;
        F2_damping = b * dot(vel_rel2, u2t) * u2t;
        F2 = F2_spring + F2_damping; % force on payload from UAV2
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

    % ===================== ROTATIONAL DYNAMICS FIXES ======================
    p1_dot = (tau_phi1   - (Iyy - Izz)*q1*r1)/Ixx;
    q1_dot = (tau_theta1 - (Izz - Ixx)*p1*r1)/Iyy;
    r1_dot = (tau_psi1   - (Ixx - Iyy)*p1*q1)/Izz;

    p2_dot = (tau_phi2   - (Iyy - Izz)*q2*r2)/Ixx;
    q2_dot = (tau_theta2 - (Izz - Ixx)*p2*r2)/Iyy;
    r2_dot = (tau_psi2   - (Ixx - Iyy)*p2*q2)/Izz;

    p1 = p1 + p1_dot*dt;  q1 = q1 + q1_dot*dt;  r1 = r1 + r1_dot*dt;
    p2 = p2 + p2_dot*dt;  q2 = q2 + q2_dot*dt;  r2 = r2 + r2_dot*dt;

    cth1 = cos(th1);  if abs(cth1) < 1e-6, cth1 = sign(cth1)*1e-6; end
    Winv1 = [ 1,  sin(phi1)*tan(th1),  cos(phi1)*tan(th1);
              0,  cos(phi1),          -sin(phi1);
              0,  sin(phi1)/cth1,      cos(phi1)/cth1 ];
    eul1_dot = Winv1 * [p1; q1; r1];
    phi1 = phi1 + eul1_dot(1)*dt;
    th1  = th1  + eul1_dot(2)*dt;
    psi1 = psi1 + eul1_dot(3)*dt;

    % UAV2
    cth2 = cos(th2);  if abs(cth2) < 1e-6, cth2 = sign(cth2)*1e-6; end
    Winv2 = [ 1,  sin(phi2)*tan(th2),  cos(phi2)*tan(th2);
              0,  cos(phi2),          -sin(phi2);
              0,  sin(phi2)/cth2,      cos(phi2)/cth2 ];
    eul2_dot = Winv2 * [p2; q2; r2];
    phi2 = phi2 + eul2_dot(1)*dt;
    th2  = th2  + eul2_dot(2)*dt;
    psi2 = psi2 + eul2_dot(3)*dt;
    % =================== END ROTATIONAL DYNAMICS FIXES ====================

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

% Rollout simulation 
function [Xseq] = simulate_rollout_2uav(x0,U,N,nu,f_step)
    nx = numel(x0); % length of states
    Xseq = zeros(nx, N+1);
    Xseq(:,1) = x0;
    xk = x0;
    for k=1:N
        uk = U((k-1)*nu+1 : k*nu);
        xk = f_step(xk, uk);
        Xseq(:,k+1) = xk;
    end
end

% Build bounds two UAVs
lb_u = [T_min; tau_phi_min; tau_theta_min; tau_psi_min; T_min; tau_phi_min; tau_theta_min; tau_psi_min];
ub_u = [T_max; tau_phi_max; tau_theta_max; tau_psi_max; T_max; tau_phi_max; tau_theta_max; tau_psi_max];
lbU = repmat(lb_u, N, 1);
ubU = repmat(ub_u, N, 1);

% Warm start (hover thrust for each UAV) 
U0 = repmat([(mP*0.5+m)*g; 0; 0; 0; (mP*0.5+m)*g; 0; 0; 0], N, 1);

%% MPC loop ---------------------------------------------------------------
tol_vel = 0.05;   % Stop when payload velocity < 5 cm/s
tol_uav_vel = 0.1; % Stop when both UAV velocities < 10 cm/s
k = 1;

% Initialize timing variables for MPC solve time measurement
total_solve_time = 0;
solve_times = zeros(MAX_STEPS, 1);

fprintf('Activating MPC...\n');

while k <= MAX_STEPS
    % Display progress
    payload_vel = norm(x([26 28 30])); 
    uav1_vel = norm(x([2 4 6])); 
    uav2_vel = norm(x([14 16 18])); 
    
    fprintf('Step %d/%d | Payload vel: %.3f m/s | UAV1 vel: %.3f m/s | UAV2 vel: %.3f m/s\n', ...
        k, MAX_STEPS, payload_vel, uav1_vel, uav2_vel);
    
    obj = @(U) stage_cost_2uav(x,U,N,nu,rollout,w_u);

    nonlcon = @(U) nl_constr_angles_2uav(x,U,N,nu,rollout,phi_max,theta_max);

    % Start timing for MPC solve
    solve_start_time = tic;
    
    % Solve NMPC
    [Uopt,~,exitflag] = fmincon(obj, U0, [],[],[],[], lbU, ubU, nonlcon, opts);
    
    % End timing for MPC solve and store
    solve_times(k) = toc(solve_start_time);
    total_solve_time = total_solve_time + solve_times(k);
    
    if exitflag <= 0
        warning('fmincon failed at step %d (exitflag=%d). Using previous control.', k, exitflag);
        Uopt = U0;
    end

    [Xpred] = rollout(x, Uopt);
    pred_traj{k} = Xpred;

    % Apply first control (both UAVs)
    u0 = Uopt(1:nu);
    x  = f_step(x, u0);

    % Shift warm start take 1st one and repeat the last one again.
    U0 = [Uopt(nu+1:end); Uopt(end-nu+1:end)]; 

    % Log
    logU(:,k)   = u0;
    logX(:,k+1) = x;

    % Check convergence based on stabilization (not position)
    % Don't allow convergence on the first step (k=1) to ensure simulation runs
    if k > 1 && payload_vel < tol_vel && uav1_vel < tol_uav_vel && uav2_vel < tol_uav_vel
        fprintf('\n*** STABILIZATION ACHIEVED at step %d! ***\n', k);
        fprintf('Payload velocity: %.4f m/s (< %.2f m/s threshold)\n', payload_vel, tol_vel);
        fprintf('UAV velocities: %.4f, %.4f m/s (< %.2f m/s threshold)\n', uav1_vel, uav2_vel, tol_uav_vel);
        break;
    end
    k = k + 1;
end

if k > MAX_STEPS
    fprintf('\n*** Reached maximum steps (%d) without full stabilization ***\n', MAX_STEPS);
    payload_vel_final = norm(x([26 28 30]));
    fprintf('Final payload velocity: %.4f m/s\n', payload_vel_final);
end

steps_used = min(k, MAX_STEPS);

% Display MPC timing results
fprintf('\n=== MPC SOLVE TIME ANALYSIS ===\n');
fprintf('Total MPC solve time: %.4f seconds\n', total_solve_time);
fprintf('Average solve time per step: %.4f seconds\n', total_solve_time / steps_used);
fprintf('Min solve time: %.4f seconds\n', min(solve_times(1:steps_used)));
fprintf('Max solve time: %.4f seconds\n', max(solve_times(1:steps_used)));
fprintf('Steps completed: %d\n', steps_used);

% Extract trajectory data for plotting
t = (0:steps_used)*dt;
% UAV1 trajectory
N1_traj = logX(1,1:steps_used+1);
E1_traj = logX(3,1:steps_used+1);
D1_traj = logX(5,1:steps_used+1);
phi1_traj   = logX(7,1:steps_used+1);
theta1_traj = logX(9,1:steps_used+1);
psi1_traj   = logX(11,1:steps_used+1);

% UAV2 trajectory (starts at index 13)
N2_traj = logX(13,1:steps_used+1);
E2_traj = logX(15,1:steps_used+1);
D2_traj = logX(17,1:steps_used+1);
phi2_traj   = logX(19,1:steps_used+1);
theta2_traj = logX(21,1:steps_used+1);
psi2_traj   = logX(23,1:steps_used+1);

% Payload trajectory (last 6 entries)
Np_traj = logX(25,1:steps_used+1);
Ep_traj = logX(27,1:steps_used+1);
Dp_traj = logX(29,1:steps_used+1);

%% ======================= Visualization Section =========================
% %% Plots ------------------------------------------------------------------
figure('Name','Position (NED) - UAVs and Payload');
subplot(3,1,1);
plot(t,N1_traj,'b-','LineWidth',1.6); hold on;
plot(t,N2_traj,'g-','LineWidth',1.6);
plot(t,Np_traj,'r--','LineWidth',1.6);
ylabel('N [m]'); grid on;
legend('UAV1','UAV2','Payload','Location','best');

subplot(3,1,2);
plot(t,E1_traj,'b-','LineWidth',1.6); hold on;
plot(t,E2_traj,'g-','LineWidth',1.6);
plot(t,Ep_traj,'r--','LineWidth',1.6);
ylabel('E [m]'); grid on;
legend('UAV1','UAV2','Payload','Location','best');

subplot(3,1,3);
plot(t,D1_traj,'b-','LineWidth',1.6); hold on;
plot(t,D2_traj,'g-','LineWidth',1.6);
plot(t,Dp_traj,'r--','LineWidth',1.6);
ylabel('D [m]'); xlabel('time [s]'); grid on;
legend('UAV1','UAV2','Payload','Location','best');

% Payload Velocity Plot
figure('Name','Payload Velocity Components');
% Extract payload velocities
vNp_traj = logX(26,1:steps_used+1);
vEp_traj = logX(28,1:steps_used+1);
vDp_traj = logX(30,1:steps_used+1);

subplot(3,1,1);
plot(t, vNp_traj, 'r-', 'LineWidth', 1.6);
ylabel('v_N [m/s]'); grid on;
title('Payload Velocity - North Component');

subplot(3,1,2);
plot(t, vEp_traj, 'r-', 'LineWidth', 1.6);
ylabel('v_E [m/s]'); grid on;
title('Payload Velocity - East Component');

subplot(3,1,3);
plot(t, vDp_traj, 'r-', 'LineWidth', 1.6);
ylabel('v_D [m/s]'); xlabel('time [s]'); grid on;
title('Payload Velocity - Down Component');

% Total payload velocity magnitude
figure('Name','Payload Velocity Magnitude');
payload_vel_mag = sqrt(vNp_traj.^2 + vEp_traj.^2 + vDp_traj.^2);
plot(t, payload_vel_mag, 'r-', 'LineWidth', 2);
ylabel('|v_{payload}| [m/s]'); xlabel('time [s]'); grid on;
title('Total Payload Velocity Magnitude');

figure('Name','Attitude (UAV1 & UAV2)');
subplot(3,1,1); plot(t,rad2deg(phi1_traj),'b','LineWidth',1.6); hold on; plot(t,rad2deg(phi2_traj),'g','LineWidth',1.2); ylabel('\phi [deg]'); grid on; legend('UAV1','UAV2');
subplot(3,1,2); plot(t,rad2deg(theta1_traj),'b','LineWidth',1.6); hold on; plot(t,rad2deg(theta2_traj),'g','LineWidth',1.2); ylabel('\theta [deg]'); grid on;
subplot(3,1,3); plot(t,rad2deg(psi1_traj),'b','LineWidth',1.6); hold on; plot(t,rad2deg(psi2_traj),'g','LineWidth',1.2); ylabel('\psi [deg]'); xlabel('time [s]'); grid on;

figure('Name','Thrust Controls');
tt = (0:steps_used-1)*dt;
subplot(2,1,1);
plot(tt, logU(1,1:steps_used),'b-','LineWidth',2);
ylabel('Thrust [N]'); grid on;
legend('UAV1 Thrust (T1)', 'Location', 'best');
title('UAV1 Thrust Command');

subplot(2,1,2);
plot(tt, logU(5,1:steps_used),'g-','LineWidth',2);
ylabel('Thrust [N]'); xlabel('time [s]'); grid on;
legend('UAV2 Thrust (T2)', 'Location', 'best');
title('UAV2 Thrust Command');

figure('Name','Torque Controls');
subplot(3,1,1);
plot(tt, logU(2,1:steps_used),'b-','LineWidth',1.5); hold on;
plot(tt, logU(6,1:steps_used),'g--','LineWidth',1.5);
ylabel('\tau_{\phi} [N⋅m]'); grid on;
legend('UAV1 Roll Torque', 'UAV2 Roll Torque', 'Location', 'best');
title('Roll Torques');

subplot(3,1,2);
plot(tt, logU(3,1:steps_used),'b-','LineWidth',1.5); hold on;
plot(tt, logU(7,1:steps_used),'g--','LineWidth',1.5);
ylabel('\tau_{\theta} [N⋅m]'); grid on;
legend('UAV1 Pitch Torque', 'UAV2 Pitch Torque', 'Location', 'best');
title('Pitch Torques');

subplot(3,1,3);
plot(tt, logU(4,1:steps_used),'b-','LineWidth',1.5); hold on;
plot(tt, logU(8,1:steps_used),'g--','LineWidth',1.5);
ylabel('\tau_{\psi} [N⋅m]'); xlabel('time [s]'); grid on;
legend('UAV1 Yaw Torque', 'UAV2 Yaw Torque', 'Location', 'best');
title('Yaw Torques');

%% Animation of UAV and Payload
% Create animation figure
f_anim = figure('Name', 'UAV-Payload Animation', 'Position', [100, 100, 800, 600], 'Color', 'w');
ax_anim = axes(f_anim, 'Units', 'pixels', 'Position', [50, 50, 700, 500]);
hold on; grid on; axis equal;

% Set axis limits with some padding around both trajectories
all_N = [N1_traj, N2_traj, Np_traj];
all_E = [E1_traj, E2_traj, Ep_traj];
all_D = [D1_traj, D2_traj, Dp_traj];

N_pad = max(abs(max(all_N) - min(all_N)) * 0.2, 2);
E_pad = max(abs(max(all_E) - min(all_E)) * 0.2, 2);
D_pad = max(abs(max(all_D) - min(all_D)) * 0.2, 2);

xlim([min(all_N)-N_pad, max(all_N)+N_pad]);
ylim([min(all_E)-E_pad, max(all_E)+E_pad]);
zlim([min(all_D)-D_pad, max(all_D)+D_pad]);

% In NED, positive D is downward
set(ax_anim, 'ZDir', 'reverse');
xlabel('North [m]'); ylabel('East [m]'); zlabel('Down [m]');
title('UAV-Payload Animation (NED)');

% Create trajectory lines (will update during animation)
traj1_line = plot3(N1_traj(1), E1_traj(1), D1_traj(1), 'b-', 'LineWidth', 1.5);
traj2_line = plot3(N2_traj(1), E2_traj(1), D2_traj(1), 'g-', 'LineWidth', 1.5);
trajp_line = plot3(Np_traj(1), Ep_traj(1), Dp_traj(1), 'r--', 'LineWidth', 1.5);

% Create drone-like UAV structures instead of simple markers
drone_size = 0.3;  % Size of the drone
arm_length = drone_size * 0.8;
prop_radius = drone_size * 0.4;

% UAV1 Drone body (center cross structure) - BLUE
uav1_body_x = plot3([N1_traj(1)-arm_length N1_traj(1)+arm_length], [E1_traj(1) E1_traj(1)], [D1_traj(1) D1_traj(1)], 'b-', 'LineWidth', 3);
uav1_body_y = plot3([N1_traj(1) N1_traj(1)], [E1_traj(1)-arm_length E1_traj(1)+arm_length], [D1_traj(1) D1_traj(1)], 'b-', 'LineWidth', 3);

% UAV2 Drone body (center cross structure) - GREEN  
uav2_body_x = plot3([N2_traj(1)-arm_length N2_traj(1)+arm_length], [E2_traj(1) E2_traj(1)], [D2_traj(1) D2_traj(1)], 'g-', 'LineWidth', 3);
uav2_body_y = plot3([N2_traj(1) N2_traj(1)], [E2_traj(1)-arm_length E2_traj(1)+arm_length], [D2_traj(1) D2_traj(1)], 'g-', 'LineWidth', 3);

% Propellers (4 circles at arm ends)
theta_prop = linspace(0, 2*pi, 16);
prop_x = prop_radius * cos(theta_prop);
prop_y = prop_radius * sin(theta_prop);
prop_z = zeros(size(prop_x));

% Initialize propellers arrays for both UAVs
propellers1 = gobjects(4,1); % UAV1 propellers
propellers2 = gobjects(4,1); % UAV2 propellers

% UAV1 Propellers (BLUE)
propellers1(1) = plot3(N1_traj(1) + arm_length + prop_x, E1_traj(1) + prop_y, D1_traj(1) + prop_z, 'b-', 'LineWidth', 2);
propellers1(2) = plot3(N1_traj(1) - arm_length + prop_x, E1_traj(1) + prop_y, D1_traj(1) + prop_z, 'b-', 'LineWidth', 2);
propellers1(3) = plot3(N1_traj(1) + prop_y, E1_traj(1) + arm_length + prop_x, D1_traj(1) + prop_z, 'b-', 'LineWidth', 2);
propellers1(4) = plot3(N1_traj(1) + prop_y, E1_traj(1) - arm_length + prop_x, D1_traj(1) + prop_z, 'b-', 'LineWidth', 2);

% UAV2 Propellers (GREEN)
propellers2(1) = plot3(N2_traj(1) + arm_length + prop_x, E2_traj(1) + prop_y, D2_traj(1) + prop_z, 'g-', 'LineWidth', 2);
propellers2(2) = plot3(N2_traj(1) - arm_length + prop_x, E2_traj(1) + prop_y, D2_traj(1) + prop_z, 'g-', 'LineWidth', 2);
propellers2(3) = plot3(N2_traj(1) + prop_y, E2_traj(1) + arm_length + prop_x, D2_traj(1) + prop_z, 'g-', 'LineWidth', 2);
propellers2(4) = plot3(N2_traj(1) + prop_y, E2_traj(1) - arm_length + prop_x, D2_traj(1) + prop_z, 'g-', 'LineWidth', 2);

% Payload marker (keep simple)
payload_marker = plot3(Np_traj(1), Ep_traj(1), Dp_traj(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

tether1_line = plot3([N1_traj(1) Np_traj(1)], [E1_traj(1) Ep_traj(1)], [D1_traj(1) Dp_traj(1)], 'k-', 'LineWidth', 2);
tether2_line = plot3([N2_traj(1) Np_traj(1)], [E2_traj(1) Ep_traj(1)], [D2_traj(1) Dp_traj(1)], 'k-', 'LineWidth', 2);

% Create UAV orientation lines (for visualizing attitude) for UAV1 and UAV2
uav_size = 0.5;
% UAV1 orientation quivers (solid)
roll1_line = quiver3(N1_traj(1), E1_traj(1), D1_traj(1), 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
pitch1_line = quiver3(N1_traj(1), E1_traj(1), D1_traj(1), 0, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
yaw1_line = quiver3(N1_traj(1), E1_traj(1), D1_traj(1), 0, 0, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% UAV2 orientation quivers (dashed, thinner)
roll2_line = quiver3(N2_traj(1), E2_traj(1), D2_traj(1), 0, 0, 0, 'r--', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
pitch2_line = quiver3(N2_traj(1), E2_traj(1), D2_traj(1), 0, 0, 0, 'g--', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
yaw2_line = quiver3(N2_traj(1), E2_traj(1), D2_traj(1), 0, 0, 0, 'b--', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

% % Add legend (show UAV markers, payload and tethers and UAV1 attitude)
% legend([uav1_body_x, uav2_body_x, payload_marker, tether1_line, tether2_line, roll1_line, pitch1_line, yaw1_line], ...
%     'UAV1', 'UAV2', 'Payload', 'Tether1', 'Tether2', 'Roll1', 'Pitch1', 'Yaw1', ...
%     'Location', 'best');

legend([uav1_body_x, uav2_body_x, payload_marker, tether1_line, tether2_line, ...
        roll1_line, pitch1_line, yaw1_line, roll2_line, pitch2_line, yaw2_line], ...
    'UAV1', 'UAV2', 'Payload', 'Tether1', 'Tether2', ...
    'UAV1 X-axis (Roll)', 'UAV1 Y-axis (Pitch)', 'UAV1 Z-axis (Yaw)', ...
    'UAV2 X-axis (Roll)', 'UAV2 Y-axis (Pitch)', 'UAV2 Z-axis (Yaw)', ...
    'Location', 'best');
% Create video writer
videoName = 'uav_stabilization_animation.mp4';
v = VideoWriter(videoName, 'MPEG-4');
v.FrameRate = 20;  % Adjust speed (frames per second)
open(v);

% Animation loop
for i = 1:length(t)
    % Update trajectory lines
    set(traj1_line, 'XData', N1_traj(1:i), 'YData', E1_traj(1:i), 'ZData', D1_traj(1:i));
    set(traj2_line, 'XData', N2_traj(1:i), 'YData', E2_traj(1:i), 'ZData', D2_traj(1:i));
    set(trajp_line, 'XData', Np_traj(1:i), 'YData', Ep_traj(1:i), 'ZData', Dp_traj(1:i));
    
    % Update UAV1 drone body position and orientation
    current_pos1 = [N1_traj(i), E1_traj(i), D1_traj(i)];
    current_att1 = [phi1_traj(i), theta1_traj(i), psi1_traj(i)]; % [roll, pitch, yaw]
    
    % Rotation matrix for UAV1 orientation
    R1 = angle2dcm(current_att1(3), current_att1(2), current_att1(1), 'ZYX');
    
    % Update UAV1 drone body arms
    arm_length = 0.24; % 0.8 * 0.3
    arm_x_local = [-arm_length, arm_length, 0, 0];
    arm_y_local = [0, 0, -arm_length, arm_length];
    arm_z_local = [0, 0, 0, 0];
    
    % Rotate arms based on UAV1 attitude
    arm_points1 = R1 * [arm_x_local; arm_y_local; arm_z_local];
    set(uav1_body_x, 'XData', [current_pos1(1) + arm_points1(1,1), current_pos1(1) + arm_points1(1,2)], ...
                     'YData', [current_pos1(2) + arm_points1(2,1), current_pos1(2) + arm_points1(2,2)], ...
                     'ZData', [current_pos1(3) + arm_points1(3,1), current_pos1(3) + arm_points1(3,2)]);
    set(uav1_body_y, 'XData', [current_pos1(1) + arm_points1(1,3), current_pos1(1) + arm_points1(1,4)], ...
                     'YData', [current_pos1(2) + arm_points1(2,3), current_pos1(2) + arm_points1(2,4)], ...
                     'ZData', [current_pos1(3) + arm_points1(3,3), current_pos1(3) + arm_points1(3,4)]);
    
    % Update UAV2 drone body position and orientation
    current_pos2 = [N2_traj(i), E2_traj(i), D2_traj(i)];
    current_att2 = [phi2_traj(i), theta2_traj(i), psi2_traj(i)]; % [roll, pitch, yaw]
    
    % Rotation matrix for UAV2 orientation
    R2 = angle2dcm(current_att2(3), current_att2(2), current_att2(1), 'ZYX');
    
    % Rotate arms based on UAV2 attitude
    arm_points2 = R2 * [arm_x_local; arm_y_local; arm_z_local];
    set(uav2_body_x, 'XData', [current_pos2(1) + arm_points2(1,1), current_pos2(1) + arm_points2(1,2)], ...
                     'YData', [current_pos2(2) + arm_points2(2,1), current_pos2(2) + arm_points2(2,2)], ...
                     'ZData', [current_pos2(3) + arm_points2(3,1), current_pos2(3) + arm_points2(3,2)]);
    set(uav2_body_y, 'XData', [current_pos2(1) + arm_points2(1,3), current_pos2(1) + arm_points2(1,4)], ...
                     'YData', [current_pos2(2) + arm_points2(2,3), current_pos2(2) + arm_points2(2,4)], ...
                     'ZData', [current_pos2(3) + arm_points2(3,3), current_pos2(3) + arm_points2(3,4)]);
    
    % Update UAV1 propellers at arm ends
    prop_radius = 0.12; % 0.4 * 0.3
    theta_prop = linspace(0, 2*pi, 16);
    prop_x_circle = prop_radius * cos(theta_prop);
    prop_y_circle = prop_radius * sin(theta_prop);
    prop_z_circle = zeros(size(prop_x_circle));
    
    % UAV1 Propeller positions in body frame
    prop_positions_local = [arm_length, -arm_length, 0, 0; 
                           0, 0, arm_length, -arm_length; 
                           0, 0, 0, 0];
    prop_positions_global1 = R1 * prop_positions_local;
    
    for j = 1:4
        prop_center1 = current_pos1 + prop_positions_global1(:,j)';
        % Create propeller circle in local frame then transform
        prop_circle_local = [prop_x_circle; prop_y_circle; prop_z_circle];
        prop_circle_global1 = R1 * prop_circle_local;
        
        set(propellers1(j), 'XData', prop_center1(1) + prop_circle_global1(1,:), ...
                           'YData', prop_center1(2) + prop_circle_global1(2,:), ...
                           'ZData', prop_center1(3) + prop_circle_global1(3,:));
    end
    
    % Update UAV2 propellers at arm ends
    prop_positions_global2 = R2 * prop_positions_local;
    
    for j = 1:4
        prop_center2 = current_pos2 + prop_positions_global2(:,j)';
        % Create propeller circle in local frame then transform
        prop_circle_global2 = R2 * prop_circle_local;
        
        set(propellers2(j), 'XData', prop_center2(1) + prop_circle_global2(1,:), ...
                           'YData', prop_center2(2) + prop_circle_global2(2,:), ...
                           'ZData', prop_center2(3) + prop_circle_global2(3,:));
    end
    
    % Update payload position
    set(payload_marker, 'XData', Np_traj(i), 'YData', Ep_traj(i), 'ZData', Dp_traj(i));
    
    % Update tether lines
    set(tether1_line, 'XData', [N1_traj(i) Np_traj(i)], 'YData', [E1_traj(i) Ep_traj(i)], 'ZData', [D1_traj(i) Dp_traj(i)]);
    set(tether2_line, 'XData', [N2_traj(i) Np_traj(i)], 'YData', [E2_traj(i) Ep_traj(i)], 'ZData', [D2_traj(i) Dp_traj(i)]);
    
    % Update orientation lines based on Euler angles for UAV1
    phi1 = phi1_traj(i); theta1 = theta1_traj(i); psi1 = psi1_traj(i);
    x1_ned = R1 * (uav_size*[1;0;0]);
    y1_ned = R1 * (uav_size*[0;1;0]);
    z1_ned = R1 * (uav_size*[0;0;1]);
    set(roll1_line, 'XData', N1_traj(i), 'YData', E1_traj(i), 'ZData', D1_traj(i), ...
                  'UData', x1_ned(1), 'VData', x1_ned(2), 'WData', x1_ned(3));
    set(pitch1_line, 'XData', N1_traj(i), 'YData', E1_traj(i), 'ZData', D1_traj(i), ...
                   'UData', y1_ned(1), 'VData', y1_ned(2), 'WData', y1_ned(3));
    set(yaw1_line, 'XData', N1_traj(i), 'YData', E1_traj(i), 'ZData', D1_traj(i), ...
                 'UData', z1_ned(1), 'VData', z1_ned(2), 'WData', z1_ned(3));

    % Update orientation for UAV2
    phi2 = phi2_traj(i); theta2 = theta2_traj(i); psi2 = psi2_traj(i);
    x2_ned = R2 * (uav_size*[1;0;0]);
    y2_ned = R2 * (uav_size*[0;1;0]);
    z2_ned = R2 * (uav_size*[0;0;1]);
    set(roll2_line, 'XData', N2_traj(i), 'YData', E2_traj(i), 'ZData', D2_traj(i), ...
                  'UData', x2_ned(1), 'VData', x2_ned(2), 'WData', x2_ned(3));
    set(pitch2_line, 'XData', N2_traj(i), 'YData', E2_traj(i), 'ZData', D2_traj(i), ...
                   'UData', y2_ned(1), 'VData', y2_ned(2), 'WData', y2_ned(3));
    set(yaw2_line, 'XData', N2_traj(i), 'YData', E2_traj(i), 'ZData', D2_traj(i), ...
                 'UData', z2_ned(1), 'VData', z2_ned(2), 'WData', z2_ned(3));

    % Add time label and tether length info for both tethers
    tether_length1 = norm([N1_traj(i) - Np_traj(i); E1_traj(i) - Ep_traj(i); D1_traj(i) - Dp_traj(i)]);
    tether_length2 = norm([N2_traj(i) - Np_traj(i); E2_traj(i) - Ep_traj(i); D2_traj(i) - Dp_traj(i)]);
    time_text = sprintf('Time: %.2f s, Tether1: %.2f m, Tether2: %.2f m', t(i), tether_length1, tether_length2);
    title(['UAV-Payload Animation (NED) - ', time_text]);
    
    % Capture frame
    drawnow;
    frame = getframe(ax_anim);
    writeVideo(v, frame);
end

% Close video file
close(v);
fprintf('Animation saved as: %s\n', videoName);

fprintf('\n=== SIMULATION COMPLETE ===\n');

%  comment this line if it caused error
tileFigs(2,3)