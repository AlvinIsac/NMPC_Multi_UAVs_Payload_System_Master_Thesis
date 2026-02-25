clear; clc; close all;

% Import CasADi
import casadi.*

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
MAX_STEPS = 450;  
nx = 30; nu = 8;

% Target position for tracking phase
Ndes = -1; Edes = 1; Ddes = -5.5;                                 

w_u  = diag([0.03 0.015 0.015 0.015]);                 

%% Constraints
phi_max   = deg2rad(45);
theta_max = deg2rad(45);
psi_max   = pi;                       

T_min = 0.0;   
T_max = 34.0;  

tau_phi_min   = -0.2; tau_phi_max   = 0.2;
tau_theta_min = -0.2; tau_theta_max = 0.2;
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
start_Ep    = 1.2;    % try the swing front or back 0.8 or -0.8
start_vEp   = 0.1;               
start_Dp    = pD1 + L ;          
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

%% CasADi Setup for MPC
% Create optimization problem
opti = casadi.Opti();

% Decision variables
X = opti.variable(nx, N+1);  % State trajectory
U = opti.variable(nu, N);    % Control sequence

% Parameters
x0_param = opti.parameter(nx, 1);  % Initial state
phase_param = opti.parameter(1, 1); % Phase parameter: 0=stabilization, 1=tracking

%% CasADi Dynamics Function
function xnext = step_dynamics_2uav_casadi(x, u, Ixx, Iyy, Izz, dt, m, mP, L, k, b, g)
    % Extract UAV1, UAV2, and payload states
    x1 = x(1:12); x2 = x(13:24); xp = x(25:30);
    
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
    r1p = pp_vec - p1_vec; d1 = norm_2(r1p);
    u1t = r1p / fmax(d1, 1e-6);
    F1_spring = k*(d1 - L)*u1t;
    vel_rel1 = vp_vec - v1_vec;
    F1_damping = b * dot(vel_rel1, u1t) * u1t;
    F1 = F1_spring + F1_damping;
    
    r2p = pp_vec - p2_vec; d2 = norm_2(r2p);
    u2t = r2p / fmax(d2, 1e-6);
    F2_spring = k*(d2 - L)*u2t;
    vel_rel2 = vp_vec - v2_vec;
    F2_damping = b * dot(vel_rel2, u2t) * u2t;
    F2 = F2_spring + F2_damping;
    
    % Translational accelerations (NED)
    aN1 = -(T1/m)*(sin(phi1)*sin(psi1) + cos(phi1)*sin(th1)*cos(psi1)) + F1(1)/m;
    aE1 = -(T1/m)*(cos(phi1)*sin(th1)*sin(psi1) - sin(phi1)*cos(psi1)) + F1(2)/m;
    aD1 = -(T1/m)*(cos(phi1)*cos(th1)) + g + F1(3)/m;
    
    aN2 = -(T2/m)*(sin(phi2)*sin(psi2) + cos(phi2)*sin(th2)*cos(psi2)) + F2(1)/m;
    aE2 = -(T2/m)*(cos(phi2)*sin(th2)*sin(psi2) - sin(phi2)*cos(psi2)) + F2(2)/m;
    aD2 = -(T2/m)*(cos(phi2)*cos(th2)) + g + F2(3)/m;
    
    % Rotational dynamics
    p1_dot = (tau_phi1 - (Iyy - Izz)*q1*r1)/Ixx;
    q1_dot = (tau_theta1 - (Izz - Ixx)*p1*r1)/Iyy;
    r1_dot = (tau_psi1 - (Ixx - Iyy)*p1*q1)/Izz;
    
    p2_dot = (tau_phi2 - (Iyy - Izz)*q2*r2)/Ixx;
    q2_dot = (tau_theta2 - (Izz - Ixx)*p2*r2)/Iyy;
    r2_dot = (tau_psi2 - (Ixx - Iyy)*p2*q2)/Izz;
    
    % Integrate body rates
    p1_new = p1 + p1_dot*dt;  q1_new = q1 + q1_dot*dt;  r1_new = r1 + r1_dot*dt;
    p2_new = p2 + p2_dot*dt;  q2_new = q2 + q2_dot*dt;  r2_new = r2 + r2_dot*dt;
    
    % Convert body rates to Euler angle rates
    cth1 = cos(th1); cth1 = max(abs(cth1), 1e-6) * sign(cth1);
    eul1_dot1 = p1_new + sin(phi1)*tan(th1)*q1_new + cos(phi1)*tan(th1)*r1_new;
    eul1_dot2 = cos(phi1)*q1_new - sin(phi1)*r1_new;
    eul1_dot3 = sin(phi1)/cth1*q1_new + cos(phi1)/cth1*r1_new;
    
    cth2 = cos(th2); cth2 = max(abs(cth2), 1e-6) * sign(cth2);
    eul2_dot1 = p2_new + sin(phi2)*tan(th2)*q2_new + cos(phi2)*tan(th2)*r2_new;
    eul2_dot2 = cos(phi2)*q2_new - sin(phi2)*r2_new;
    eul2_dot3 = sin(phi2)/cth2*q2_new + cos(phi2)/cth2*r2_new;
    
    % Integrate Euler angles
    phi1_new = phi1 + eul1_dot1*dt;
    th1_new = th1 + eul1_dot2*dt;
    psi1_new = psi1 + eul1_dot3*dt;
    
    phi2_new = phi2 + eul2_dot1*dt;
    th2_new = th2 + eul2_dot2*dt;
    psi2_new = psi2 + eul2_dot3*dt;
    
    % Integrate UAV translations
    vN1_new = vN1 + aN1*dt;   N1_new = N1 + vN1_new*dt;
    vE1_new = vE1 + aE1*dt;   E1_new = E1 + vE1_new*dt;
    vD1_new = vD1 + aD1*dt;   D1_new = D1 + vD1_new*dt;
    
    vN2_new = vN2 + aN2*dt;   N2_new = N2 + vN2_new*dt;
    vE2_new = vE2 + aE2*dt;   E2_new = E2 + vE2_new*dt;
    vD2_new = vD2 + aD2*dt;   D2_new = D2 + vD2_new*dt;
    
    % Payload dynamics
    aNp = -(F1(1) + F2(1))/mP;
    aEp = -(F1(2) + F2(2))/mP;
    aDp = -(F1(3) + F2(3))/mP + g;
    
    % Integrate payload states
    vNp_new = vNp + aNp*dt;  Np_new = Np + vNp_new*dt;
    vEp_new = vEp + aEp*dt;  Ep_new = Ep + vEp_new*dt;
    vDp_new = vDp + aDp*dt;  Dp_new = Dp + vDp_new*dt;
    
    % Pack next state
    x1_next = [N1_new; vN1_new; E1_new; vE1_new; D1_new; vD1_new; phi1_new; p1_new; th1_new; q1_new; psi1_new; r1_new];
    x2_next = [N2_new; vN2_new; E2_new; vE2_new; D2_new; vD2_new; phi2_new; p2_new; th2_new; q2_new; psi2_new; r2_new];
    xp_next = [Np_new; vNp_new; Ep_new; vEp_new; Dp_new; vDp_new];
    
    xnext = [x1_next; x2_next; xp_next];
end

% Create CasADi dynamics function
x_sym = SX.sym('x', nx);
u_sym = SX.sym('u', nu);
f_casadi = Function('f', {x_sym, u_sym}, {step_dynamics_2uav_casadi(x_sym, u_sym, Ixx, Iyy, Izz, dt, m, mP, L, k, b, g)});

%% Setup MPC optimization problem
% Initial condition constraint
opti.subject_to(X(:,1) == x0_param);

% Dynamics constraints
for k = 1:N
    opti.subject_to(X(:,k+1) == f_casadi(X(:,k), U(:,k)));
end

% Control bounds
for k = 1:N
    % UAV1 bounds
    opti.subject_to(U(1,k) >= T_min);
    opti.subject_to(U(1,k) <= T_max);
    opti.subject_to(U(2,k) >= tau_phi_min);
    opti.subject_to(U(2,k) <= tau_phi_max);
    opti.subject_to(U(3,k) >= tau_theta_min);
    opti.subject_to(U(3,k) <= tau_theta_max);
    opti.subject_to(U(4,k) >= tau_psi_min);
    opti.subject_to(U(4,k) <= tau_psi_max);
    
    % UAV2 bounds
    opti.subject_to(U(5,k) >= T_min);
    opti.subject_to(U(5,k) <= T_max);
    opti.subject_to(U(6,k) >= tau_phi_min);
    opti.subject_to(U(6,k) <= tau_phi_max);
    opti.subject_to(U(7,k) >= tau_theta_min);
    opti.subject_to(U(7,k) <= tau_theta_max);
    opti.subject_to(U(8,k) >= tau_psi_min);
    opti.subject_to(U(8,k) <= tau_psi_max);
end

% Angle constraints
for k = 1:N+1
    % UAV1 angle limits
    opti.subject_to(X(7,k) >= -phi_max);
    opti.subject_to(X(7,k) <= phi_max);
    opti.subject_to(X(9,k) >= -theta_max);
    opti.subject_to(X(9,k) <= theta_max);
    
    % UAV2 angle limits  
    opti.subject_to(X(19,k) >= -phi_max);
    opti.subject_to(X(19,k) <= phi_max);
    opti.subject_to(X(21,k) >= -theta_max);
    opti.subject_to(X(21,k) <= theta_max);
end

% Two-phase cost function
J = 0;
dis_between_uav = 1.2;

for k = 1:N
    xk = X(:,k);
    uk = U(:,k);
    
    % Extract states
    N1 = xk(1); vN1 = xk(2); E1 = xk(3); vE1 = xk(4); D1 = xk(5); vD1 = xk(6);
    phi1 = xk(7); p1 = xk(8); theta1 = xk(9); q1 = xk(10); psi1 = xk(11); r1 = xk(12);
    
    N2 = xk(13); vN2 = xk(14); E2 = xk(15); vE2 = xk(16); D2 = xk(17); vD2 = xk(18);
    phi2 = xk(19); p2 = xk(20); theta2 = xk(21); q2 = xk(22); psi2 = xk(23); r2 = xk(24);
    
    Np = xk(25); vNp = xk(26); Ep = xk(27); vEp = xk(28); Dp = xk(29); vDp = xk(30);
    
    u1 = uk(1:4); u2 = uk(5:8);
    
    % TRACKING COST - includes position tracking and payload safety
    tracking_cost = 0;
    max_individual_vel = 0.4;
    
    % 1. Formation center tracking to target
    uav_center_N = (N1 + N2) / 2;
    uav_center_E = (E1 + E2) / 2;
    uav_center_D = (D1 + D2) / 2;
    
    % Smooth position tracking
    dist_to_target = sqrt((uav_center_N - Ndes)^2 + (uav_center_E - Edes)^2 + (uav_center_D - Ddes)^2);
    position_weight = 12.0 * (1.0 + 1.5*exp(-1.5*dist_to_target));
    tracking_cost = tracking_cost + position_weight * ((uav_center_N - Ndes)^2 + (uav_center_E - Edes)^2 + (uav_center_D - Ddes)^2);
    
    % 2. Maintain exact UAV horizontal separation
    horizontal_separation = sqrt((N1 - N2)^2 + (E1 - E2)^2);
    separation_error = horizontal_separation - dis_between_uav;
    tracking_cost = tracking_cost + 50.0 * separation_error^2;
    
    % 3. Maintain exact same altitude
    altitude_difference = D1 - D2;
    tracking_cost = tracking_cost + 120.0 * altitude_difference^2;
    
    % 4. Keep individual UAV velocities small
    vel1_magnitude = sqrt(vN1^2 + vE1^2 + vD1^2);
    vel2_magnitude = sqrt(vN2^2 + vE2^2 + vD2^2);
    tracking_cost = tracking_cost + 25.0 * fmax(0, vel1_magnitude - max_individual_vel)^2;
    tracking_cost = tracking_cost + 25.0 * fmax(0, vel2_magnitude - max_individual_vel)^2;
    tracking_cost = tracking_cost + 8.0 * (vel1_magnitude^2 + vel2_magnitude^2);
    
    % 5. Perfect velocity coordination between UAVs
    tracking_cost = tracking_cost + 30.0 * ((vN1 - vN2)^2 + (vE1 - vE2)^2 + (vD1 - vD2)^2);
    
    % 6. Smooth formation movement toward target
    max_formation_vel = 0.3;
    error_N = Ndes - uav_center_N;
    error_E = Edes - uav_center_E;
    error_D = Ddes - uav_center_D;
    
    desired_vel_N = max_formation_vel * tanh(1.0 * error_N);
    desired_vel_E = max_formation_vel * tanh(1.0 * error_E);
    desired_vel_D = max_formation_vel * tanh(1.0 * error_D);
    
    avg_vel_N = (vN1 + vN2) / 2;
    avg_vel_E = (vE1 + vE2) / 2;
    avg_vel_D = (vD1 + vD2) / 2;
    
    tracking_cost = tracking_cost + 10.0 * ((avg_vel_N - desired_vel_N)^2 + (avg_vel_E - desired_vel_E)^2 + (avg_vel_D - desired_vel_D)^2);
    
    % 7. Payload oscillation damping
    tracking_cost = tracking_cost + 80.0 * (vNp^2 + vEp^2 + vDp^2);
    
    % 7b. Additional payload stability
    if k > 1
        xk_prev = X(:,k-1);
        vNp_prev = xk_prev(26); vEp_prev = xk_prev(28); vDp_prev = xk_prev(30);
        payload_accel_N = (vNp - vNp_prev);
        payload_accel_E = (vEp - vEp_prev);
        payload_accel_D = (vDp - vDp_prev);
        tracking_cost = tracking_cost + 40.0 * (payload_accel_N^2 + payload_accel_E^2 + payload_accel_D^2);
    end
    
    % 8. Keep payload naturally centered
    payload_center_N = (N1 + N2) / 2;
    payload_center_E = (E1 + E2) / 2;
    tracking_cost = tracking_cost + 15.0 * ((Np - payload_center_N)^2 + (Ep - payload_center_E)^2);
    
    % 9. Attitude stability
    tracking_cost = tracking_cost + 5.0 * (phi1^2 + theta1^2 + phi2^2 + theta2^2);
    
    % 10. Angular velocity minimization
    tracking_cost = tracking_cost + 3.0 * (p1^2 + q1^2 + r1^2 + p2^2 + q2^2 + r2^2);
    
    % 11. Control effort
    tracking_cost = tracking_cost + u1'*w_u*u1 + u2'*w_u*u2;
    
    % 12. Formation orientation consistency
    % yaw_difference = psi1 - psi2;
    % Normalize yaw difference to [-pi, pi]
    % yaw_difference = atan2(sin(yaw_difference), cos(yaw_difference));
    % tracking_cost = tracking_cost + 8.0 * yaw_difference^2;

    % Use only tracking cost (removed stabilization phase)
    J = J + tracking_cost;
end

% Terminal cost
xT = X(:,N+1);
N1T = xT(1); vN1T = xT(2); E1T = xT(3); vE1T = xT(4); D1T = xT(5); vD1T = xT(6);
phi1T = xT(7); theta1T = xT(9); psi1T = xT(11);
N2T = xT(13); vN2T = xT(14); E2T = xT(15); vE2T = xT(16); D2T = xT(17); vD2T = xT(18);
phi2T = xT(19); theta2T = xT(21); psi2T = xT(23);
NpT = xT(25); vNpT = xT(26); EpT = xT(27); vEpT = xT(28); DpT = xT(29); vDpT = xT(30);

% TRACKING TERMINAL COST
track_terminal_cost = 0;

% Terminal position accuracy
uav_center_NT = (N1T + N2T) / 2;
uav_center_ET = (E1T + E2T) / 2;
uav_center_DT = (D1T + D2T) / 2;
track_terminal_cost = track_terminal_cost + 150 * ((uav_center_NT - Ndes)^2 + (uav_center_ET - Edes)^2 + (uav_center_DT - Ddes)^2);

% Terminal formation separation
horizontal_separation_T = sqrt((N1T - N2T)^2 + (E1T - E2T)^2);
separation_error_T = horizontal_separation_T - dis_between_uav;
track_terminal_cost = track_terminal_cost + 80.0 * separation_error_T^2;

% Terminal altitude coordination
altitude_difference_T = D1T - D2T;
track_terminal_cost = track_terminal_cost + 200.0 * altitude_difference_T^2;

% Terminal velocity coordination
track_terminal_cost = track_terminal_cost + 50.0 * ((vN1T - vN2T)^2 + (vE1T - vE2T)^2 + (vD1T - vD2T)^2);

% Terminal individual velocity minimization
vel1_magnitude_T = sqrt(vN1T^2 + vE1T^2 + vD1T^2);
vel2_magnitude_T = sqrt(vN2T^2 + vE2T^2 + vD2T^2);
track_terminal_cost = track_terminal_cost + 40.0 * (vel1_magnitude_T^2 + vel2_magnitude_T^2);

% Terminal payload stability
track_terminal_cost = track_terminal_cost + 25.0 * (vNpT^2 + vEpT^2 + vDpT^2);

% Terminal attitude coordination
track_terminal_cost = track_terminal_cost + 15.0 * (phi1T^2 + theta1T^2 + phi2T^2 + theta2T^2);

% Terminal yaw coordination
% yaw_difference_T = psi1T - psi2T;
% yaw_difference_T = atan2(sin(yaw_difference_T), cos(yaw_difference_T));
% track_terminal_cost = track_terminal_cost + 20.0 * yaw_difference_T^2;

% Use only tracking terminal cost (removed stabilization phase)
J = J + track_terminal_cost;

% Set objective
opti.minimize(J);

% Configure IPOPT solver
ipopt_opts = struct('print_level', 0, 'max_iter', 100);
solver_opts = struct('print_time', false);
opti.solver('ipopt', solver_opts, ipopt_opts);

% Warm start (hover thrust for each UAV)
U0 = repmat([(mP*0.5+m)*g; 0; 0; 0; (mP*0.5+m)*g; 0; 0; 0], 1, N);
X0 = repmat(x, 1, N+1);

% Set initial guess
opti.set_initial(U, U0);
opti.set_initial(X, X0);

%% MPC loop ---------------------------------------------------------------
tol_position = 0.15; % Position tolerance for reaching target (15 cm)
tol_vel = 0.05;      % Velocity tolerance for stopping (5 cm/s)
tol_uav_vel = 0.05;  % UAV velocity tolerance for stabilization (5 cm/s)
k = 1;

fprintf('Starting CasADi MPC - Tracking mode only...\n');
fprintf('Objective: Move to target position while maintaining payload safety and formation\n');

while k <= MAX_STEPS
    % Display progress
    payload_vel = norm(x([26 28 30])); 
    uav1_vel = norm(x([2 4 6])); 
    uav2_vel = norm(x([14 16 18])); 
    
    % Calculate current center position
    uav_center_N = (x(1) + x(13)) / 2;
    uav_center_E = (x(3) + x(15)) / 2;
    uav_center_D = (x(5) + x(17)) / 2;
    position_error = norm([uav_center_N - Ndes, uav_center_E - Edes, uav_center_D - Ddes]);
    
    % Display tracking progress (no phase switching - always tracking)
    fprintf('Step %d/%d [TRACKING] | Pos error: %.3f m | Payload vel: %.3f m/s | UAV vels: %.3f, %.3f m/s\n', ...
        k, MAX_STEPS, position_error, payload_vel, uav1_vel, uav2_vel);
    
    % Set current state as parameter
    opti.set_value(x0_param, x);
    
    % Solve optimization problem
    try
        sol = opti.solve();
        
        % Extract solution
        Uopt = sol.value(U);
        Xpred = sol.value(X);
        
        % fprintf('  -> CasADi solved successfully\n');
        
    catch ME
        warning('CasADi solver failed at step %d: %s. Using previous control.', k, ME.message);
        
        % Use warm start as fallback
        if k == 1
            Uopt = U0;
        else
            % Shift previous control and repeat last input
            Uopt = [Uopt(:,2:end), Uopt(:,end)];
        end
        
        % Create dummy prediction for visualization
        Xpred = repmat(x, 1, N+1);
    end
    
    % Store prediction for visualization
    pred_traj{k} = Xpred;

    % Apply first control (both UAVs)
    u0 = Uopt(:,1);
    x  = full(f_casadi(x, u0));

    % Update warm start with shifted solution
    if size(Uopt,2) >= N
        opti.set_initial(U, [Uopt(:,2:end), Uopt(:,end)]);
        opti.set_initial(X, [Xpred(:,2:end), Xpred(:,end)]);
    end

    % Log
    logU(:,k)   = u0;
    logX(:,k+1) = x;

    % Check convergence - target reached when both position and velocity criteria are met
    if position_error < tol_position && payload_vel < tol_vel
        fprintf('\n*** TARGET REACHED at step %d! ***\n', k);
        fprintf('Position error: %.4f m (< %.2f m threshold)\n', position_error, tol_position);
        fprintf('Payload velocity: %.4f m/s (< %.2f m/s threshold)\n', payload_vel, tol_vel);
        break;
    end
    
    k = k + 1;
end

if k > MAX_STEPS
    fprintf('\n*** Reached maximum steps (%d) ***\n', MAX_STEPS);
    fprintf('Final position error: %.4f m\n', position_error);
    payload_vel_final = norm(x([26 28 30]));
    fprintf('Final payload velocity: %.4f m/s\n', payload_vel_final);
end

steps_used = min(k, MAX_STEPS);

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
yline(Ndes,'k--','LineWidth',2); ylabel('N [m]'); grid on;
legend('UAV1','UAV2','Payload','Target N','Location','best');
title('North Position');

subplot(3,1,2);
plot(t,E1_traj,'b-','LineWidth',1.6); hold on;
plot(t,E2_traj,'g-','LineWidth',1.6);
plot(t,Ep_traj,'r--','LineWidth',1.6);
yline(Edes,'k--','LineWidth',2); ylabel('E [m]'); grid on;
legend('UAV1','UAV2','Payload','Target E','Location','best');
title('East Position');

subplot(3,1,3);
plot(t,D1_traj,'b-','LineWidth',1.6); hold on;
plot(t,D2_traj,'g-','LineWidth',1.6);
plot(t,Dp_traj,'r--','LineWidth',1.6);
yline(Ddes,'k--','LineWidth',2); ylabel('D [m]'); xlabel('time [s]'); grid on;
legend('UAV1','UAV2','Payload','Target D','Location','best');
title('Down Position');

% Add center position tracking plot
figure('Name','UAV Formation Center vs Target');
center_N = (N1_traj + N2_traj) / 2;
center_E = (E1_traj + E2_traj) / 2;
center_D = (D1_traj + D2_traj) / 2;

subplot(3,1,1);
plot(t, center_N, 'k-', 'LineWidth', 2); hold on;
yline(Ndes,'r--','LineWidth',2); 
ylabel('Center N [m]'); grid on;
legend('UAV Center','Target N','Location','best');
title('Formation Center vs Target Position');

subplot(3,1,2);
plot(t, center_E, 'k-', 'LineWidth', 2); hold on;
yline(Edes,'r--','LineWidth',2);
ylabel('Center E [m]'); grid on;
legend('UAV Center','Target E','Location','best');

subplot(3,1,3);
plot(t, center_D, 'k-', 'LineWidth', 2); hold on;
yline(Ddes,'r--','LineWidth',2);
ylabel('Center D [m]'); xlabel('time [s]'); grid on;
legend('UAV Center','Target D','Location','best');

figure('Name','Payload Velocity Reduction');
% Calculate payload velocity components and magnitude
payload_vel_N = logX(26,1:steps_used+1);  % North velocity
payload_vel_E = logX(28,1:steps_used+1);  % East velocity  
payload_vel_D = logX(30,1:steps_used+1);  % Down velocity
payload_vel_magnitude = sqrt(payload_vel_N.^2 + payload_vel_E.^2 + payload_vel_D.^2);

subplot(2,2,1);
plot(t, payload_vel_magnitude, 'r-', 'LineWidth', 2.5); hold on;
ylabel('Payload Speed [m/s]'); xlabel('time [s]'); grid on;
title('Payload Velocity Magnitude Over Time');
legend('Payload Speed','Location','best');

subplot(2,2,2);
plot(t, payload_vel_N, 'b-', 'LineWidth', 2); hold on;
yline(0,'k--','LineWidth',1,'Alpha',0.5); 
ylabel('North Velocity [m/s]'); xlabel('time [s]'); grid on;
title('Payload North Velocity Component');
legend('North Velocity','Zero Reference','Location','best');

subplot(2,2,3);
plot(t, payload_vel_E, 'g-', 'LineWidth', 2); hold on;
yline(0,'k--','LineWidth',1,'Alpha',0.5); 
ylabel('East Velocity [m/s]'); xlabel('time [s]'); grid on;
title('Payload East Velocity Component');
legend('East Velocity','Zero Reference','Location','best');

subplot(2,2,4);
plot(t, payload_vel_D, 'r-', 'LineWidth', 2); hold on;
yline(0,'k--','LineWidth',1,'Alpha',0.5); 
ylabel('Down Velocity [m/s]'); xlabel('time [s]'); grid on;
title('Payload Down Velocity Component');
legend('Down Velocity','Zero Reference','Location','best');

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
all_N = [N1_traj, N2_traj, Np_traj, Ndes];
all_E = [E1_traj, E2_traj, Ep_traj, Edes];
all_D = [D1_traj, D2_traj, Dp_traj, Ddes];

N_pad = max(abs(max(all_N) - min(all_N)) * 0.2, 2);
E_pad = max(abs(max(all_E) - min(all_E)) * 0.2, 2);
D_pad = max(abs(max(all_D) - min(all_D)) * 0.2, 2);

xlim([min(all_N)-N_pad, max(all_N)+N_pad]);
ylim([min(all_E)-E_pad, max(all_E)+E_pad]);
zlim([min(all_D)-D_pad, max(all_D)+D_pad]);

% In NED, positive D is downward
set(ax_anim, 'ZDir', 'reverse');
xlabel('North [m]'); ylabel('East [m]'); zlabel('Down [m]');
title('Two-Phase UAV-Payload CasADi MPC (NED)');

% Add target position marker
target_marker = plot3(Ndes, Edes, Ddes, 'mx', 'MarkerSize', 15, 'LineWidth', 4);

% Create trajectory lines (will update during animation)
traj1_line = plot3(N1_traj(1), E1_traj(1), D1_traj(1), 'b-', 'LineWidth', 1.5);
traj2_line = plot3(N2_traj(1), E2_traj(1), D2_traj(1), 'g-', 'LineWidth', 1.5);
trajp_line = plot3(Np_traj(1), Ep_traj(1), Dp_traj(1), 'r--', 'LineWidth', 1.5);

% Add formation center trajectory
center_N_traj = (N1_traj + N2_traj) / 2;
center_E_traj = (E1_traj + E2_traj) / 2;
center_D_traj = (D1_traj + D2_traj) / 2;
center_traj_line = plot3(center_N_traj(1), center_E_traj(1), center_D_traj(1), 'k:', 'LineWidth', 2);

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

% Add legend after all graphics elements are created
legend([uav1_body_x, uav2_body_x, payload_marker, tether1_line, tether2_line, target_marker, center_traj_line, roll1_line, pitch1_line, yaw1_line], ...
    'UAV1', 'UAV2', 'Payload', 'Tether1', 'Tether2', 'Target', 'Center Path', 'Roll1', 'Pitch1', 'Yaw1', ...
    'Location', 'best');

% Create video writer
videoName = 'uav_payload_tracking_animation.mp4';
v = VideoWriter(videoName, 'MPEG-4');
v.FrameRate = 20;  % Adjust speed (frames per second)
open(v);

% Animation loop
for i = 1:length(t)
    % Determine current phase based on stabilization point
    current_payload_vel = norm([logX(26,i), logX(28,i), logX(30,i)]);
    current_uav1_vel = norm([logX(2,i), logX(4,i), logX(6,i)]);
    current_uav2_vel = norm([logX(14,i), logX(16,i), logX(18,i)]);
    
    is_stabilized = (current_payload_vel < tol_vel && current_uav1_vel < tol_uav_vel && current_uav2_vel < tol_uav_vel);
    
    % Update trajectory lines
    set(traj1_line, 'XData', N1_traj(1:i), 'YData', E1_traj(1:i), 'ZData', D1_traj(1:i));
    set(traj2_line, 'XData', N2_traj(1:i), 'YData', E2_traj(1:i), 'ZData', D2_traj(1:i));
    set(trajp_line, 'XData', Np_traj(1:i), 'YData', Ep_traj(1:i), 'ZData', Dp_traj(1:i));
    
    % Update center trajectory
    set(center_traj_line, 'XData', center_N_traj(1:i), 'YData', center_E_traj(1:i), 'ZData', center_D_traj(1:i));
    
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

    % Calculate distances and errors for display
    tether_length1 = norm([N1_traj(i) - Np_traj(i); E1_traj(i) - Ep_traj(i); D1_traj(i) - Dp_traj(i)]);
    tether_length2 = norm([N2_traj(i) - Np_traj(i); E2_traj(i) - Ep_traj(i); D2_traj(i) - Dp_traj(i)]);
    
    % Distance to target
    current_center_pos = [center_N_traj(i), center_E_traj(i), center_D_traj(i)];
    distance_to_target = norm(current_center_pos - [Ndes, Edes, Ddes]);
    
    % Phase indicator
    if is_stabilized || i > 100*20  % Approximate step conversion for visualization
        phase_str = 'TRACKING';
    else
        phase_str = 'STABILIZING';
    end
    
    % Enhanced title with phase and metrics information
    time_text = sprintf('Time: %.2f s | Phase: %s | Target Dist: %.2f m | Tethers: %.2f, %.2f m', ...
        t(i), phase_str, distance_to_target, tether_length1, tether_length2);
    title(['Two-Phase UAV-Payload CasADi MPC (NED) - ', time_text]);
    
    % Capture frame
    drawnow;
    frame = getframe(ax_anim);
    writeVideo(v, frame);
end

% Close video file
close(v);
fprintf('Animation saved as: %s\n', videoName);

fprintf('\n=== TWO-PHASE CASADI SIMULATION COMPLETE ===\n');
fprintf('To replay animation: replay_anim()\n');
%  comment this line if it caused error
tileFigs(2,3)