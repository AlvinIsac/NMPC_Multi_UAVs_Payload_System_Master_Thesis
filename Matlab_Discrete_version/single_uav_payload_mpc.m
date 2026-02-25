clear; clc; close all;

%% Physical parameters
m   = 1.5;                
mP  = 1;                
g   = 9.80665;            
Ixx = 0.022; Iyy = 0.022; Izz = 0.04;  

% Tether parameters
L   = 1.0;                
k   = 50;                 
b   = 1.5;                

%% Discretization and MPC settings
dt = 0.05;        
N  = 10;          
MAX_STEPS = 250;  % simulate up to 12.5 s

% State vector x = [UAV: N vN E vE D vD φ p θ q ψ r, Payload: Np vNp Ep vEp Dp vDp]' (18x1)
nx = 18; nu = 4;

% Targets (NED)
Ndes = -5; Edes = -7; Ddes = -6;                    

%% Weights (tune for smooth control)
w_pN = 5;   w_vN = 10;    
w_pE = 5;   w_vE = 10;    
w_pD = 6;   w_vD = 10;    
w_psi= 0; w_r  = 0;              
w_u  = diag([0.08 0.05 0.05 0.02]);  
w_term = 20;                                                    

%% Constraints
phi_max   = deg2rad(45);
theta_max = deg2rad(45);
psi_max   = deg2rad(45);             

T_min = 0.0;   
T_max = 34.0;  

tau_phi_min   = -0.3; tau_phi_max   = 0.3;
tau_theta_min = -0.3; tau_theta_max = 0.3;
tau_psi_min   = -0.2; tau_psi_max   = 0.2;

lb_u = [T_min; tau_phi_min; tau_theta_min; tau_psi_min];
ub_u = [T_max; tau_phi_max; tau_theta_max; tau_psi_max]; 

%% Initial state (user-defined, full state vector)

% === User-defined initial conditions ===
N1     =  2;                
vN1    =  0.5;              
E1     = -3;                
vE1    =  0;                
D1     =  5;                
vD1    =  0;                 

phi1   = deg2rad(5);         
p1     = 0;                  
theta1 = deg2rad(0);         
q1     = 0;                  
psi1   = deg2rad(10);        
r1     = 0;                  

% Payload initial conditions
Np1    = N1;            
vNp1   = vN1;           
Ep1    = E1;            
vEp1   = vE1;           
Dp1    = D1 + L;        
vDp1   = vD1;           

% === state vector ===
x = [ N1;
      vN1;
      E1;
      vE1;
      D1;
      vD1;
      phi1;
      p1;
      theta1;
      q1;
      psi1;
      r1;
      Np1;
      vNp1;
      Ep1;
      vEp1;
      Dp1;
      vDp1 ];

%% Storage for logs
logX = zeros(nx, MAX_STEPS+1);
logU = zeros(nu, MAX_STEPS);
logX(:,1) = x;

% For visualization 
pred_traj = cell(MAX_STEPS,1);

%% fmincon options 
opts = optimoptions('fmincon','Algorithm','sqp',...
    'Display','none','MaxIterations',150,'OptimalityTolerance',1e-3,...
    'StepTolerance',1e-4,'ConstraintTolerance',1e-3);

%% Helper functions -------------------------------------------------------

f_step = @(x,u) step_dynamics(x,u,Ixx,Iyy,Izz,dt,m,g,mP,L,k,b); % one Euler step with payload

% Rollout over horizon given current x0 and control sequence U (vectorized)
rollout = @(x0,U) simulate_rollout(x0,U,N,nu,f_step);

% Cost over horizon (single-shooting) with smoothness penalty
function [J,Xseq] = stage_cost(x0,U,N,nu,Ndes,Edes,Ddes,...
    w_pN,w_vN,w_pE,w_vE,w_pD,w_vD,w_u,w_term,rollout)
    [Xseq] = rollout(x0,U);
    J = 0;
    
    % Control smoothness weight
    w_smooth = 5.0;  
    
    for k=1:N
        xk = Xseq(:,k);
        uk = U((k-1)*nu+1:k*nu);
        % Unpack
        Np=xk(1); vN=xk(2); Ep=xk(3); vE=xk(4); Dp=xk(5); vD=xk(6);
        phi=xk(7); p=xk(8); theta=xk(9); q=xk(10); psi=xk(11); r=xk(12);
        
        % Running cost
        J = J ...
          + w_pN*(Np-Ndes)^2 + w_vN*(vN-0)^2 ...
          + w_pE*(Ep-Edes)^2 + w_vE*(vE-0)^2 ...
          + w_pD*(Dp-Ddes)^2 + w_vD*(vD-0)^2 ...
          + uk'*w_u*uk;
          
        % Soft angle penalties to discourage large attitudes (helps convergence)
        J = J + 0.5*( (phi/deg2rad(50))^2 + (theta/deg2rad(50))^2 );
        
        % Control smoothness penalty (penalize rapid control changes)
        if k > 1
            uk_prev = U((k-2)*nu+1:(k-1)*nu);
            du = uk - uk_prev;
            J = J + w_smooth * (du'*du);  % Penalty on control rate
        end
    end
    % Terminal position penalty
    xT = Xseq(:,N+1);
    J = J + w_term*( (xT(1)-Ndes)^2 + (xT(3)-Edes)^2 + (xT(5)-Ddes)^2 );
end

% Nonlinear constraints: keep angles within bounds over the whole horizon
function [c,ceq] = nl_constr_angles(x0,U,N,nu,rollout,phi_max,theta_max,psi_max)
    Xseq = rollout(x0,U);
    phi   = Xseq(7,:);  theta = Xseq(9,:);  psi = Xseq(11,:);
    % Inequalities c(x) <= 0
    c = [ abs(phi)-phi_max, abs(theta)-theta_max, abs(psi)-psi_max ]';
    c = c(:);
    ceq = []; % no equality constraints here
end

% Dynamics step function with payload
function xnext = step_dynamics(x,u,Ixx,Iyy,Izz,dt,m,g,mP,L,k,b)
    % Unpack UAV states
    N=x(1); vN=x(2); E=x(3); vE=x(4); D=x(5); vD=x(6);
    phi=x(7); p=x(8); theta=x(9); q=x(10); psi=x(11); r=x(12);
    
    % Unpack Payload states
    Np=x(13); vNp=x(14); Ep=x(15); vEp=x(16); Dp=x(17); vDp=x(18);
    
    % Control inputs
    T=u(1); tau_phi=u(2); tau_theta=u(3); tau_psi=u(4);

    % UAV and Payload positions
    pos_uav = [N; E; D];
    vel_uav = [vN; vE; vD];
    pos_payload = [Np; Ep; Dp];
    vel_payload = [vNp; vEp; vDp];
    
    % Tether vector (UAV to Payload)
    r_tether = pos_payload - pos_uav;
    d_tether = norm(r_tether);
    
    % Tether forces (spring-damper)
    if d_tether > 1e-6
        unit_tether = r_tether / d_tether;
        % Spring force
        F_spring = k * (d_tether - L) * unit_tether;
        % Damping force
        vel_rel = vel_payload - vel_uav;
        F_damping = b * dot(vel_rel, unit_tether) * unit_tether;
        % Total tether force on payload
        F_tether = F_spring + F_damping;
    else
        F_tether = [0; 0; 0];
    end

    % UAV dynamics (with tether reaction force)
    aN = - (T/m)*( sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi) ) + F_tether(1)/m;
    aE = - (T/m)*( cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi) ) + F_tether(2)/m;
    aD = - (T/m)*( cos(phi)*cos(theta) ) + g + F_tether(3)/m;

    % UAV angular dynamics
    p_dot = tau_phi  / Ixx;
    q_dot = tau_theta/ Iyy;
    r_dot = tau_psi  / Izz;

    % Integrate UAV states
    vN = vN + aN*dt;   N = N + vN*dt;
    vE = vE + aE*dt;   E = E + vE*dt;
    vD = vD + aD*dt;   D = D + vD*dt;

    p = p + p_dot*dt;    phi   = phi   + p*dt;
    q = q + q_dot*dt;    theta = theta + q*dt;
    r = r + r_dot*dt;    psi   = psi   + r*dt;

    % Payload dynamics
    aNp = -F_tether(1)/mP;
    aEp = -F_tether(2)/mP;
    aDp = g - F_tether(3)/mP;
    
    % Integrate payload states
    vNp = vNp + aNp*dt;  Np = Np + vNp*dt;
    vEp = vEp + aEp*dt;  Ep = Ep + vEp*dt;
    vDp = vDp + aDp*dt;  Dp = Dp + vDp*dt;

    xnext = [N; vN; E; vE; D; vD; phi; p; theta; q; psi; r; Np; vNp; Ep; vEp; Dp; vDp];
end

% Rollout simulation
function [Xseq] = simulate_rollout(x0,U,N,nu,f_step)
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

lbU = repmat(lb_u, N, 1);
ubU = repmat(ub_u, N, 1);

% Warm start with proper total system weight
hover_thrust = (m + mP) * g;  
U0 = repmat([hover_thrust; 0; 0; 0], N, 1); % Proper hover thrust

%% MPC loop ---------------------------------------------------------------
tol_pos = 0.15;   % stop when within 15 cm in all axes
k = 1;

fprintf('\n=== Starting UAV-Payload MPC ===\n');
fprintf('Target: [%.1f, %.1f, %.1f] | Tolerance: %.2fm | Max steps: %d\n\n', Ndes, Edes, Ddes, tol_pos, MAX_STEPS);

while k <= MAX_STEPS
    % Current positions and velocities for progress tracking
    current_pos = x([1 3 5]); 
    payload_pos = x([13 15 17]);
    uav_vel = norm(x([2 4 6])); 
    payload_vel = norm(x([14 16 18])); % Payload velocity magnitude
    
    % Distance to target
    dist_to_target = norm(current_pos - [Ndes; Edes; Ddes]);
    payload_dist_to_target = norm(payload_pos - [Ndes; Edes; Ddes]);
    
    % Progress indicator every 20 steps or significant events
    if mod(k-1, 20) == 0 || k == 1
        progress_pct = (k-1) / MAX_STEPS * 100;
        fprintf('Step %3d/%d (%.0f%%) | Dist: %.2fm\n', k, MAX_STEPS, progress_pct, dist_to_target);
        
        % % comment out for the states 
        % if dist_to_target < 0.5
        %     fprintf('APPROACHING\n');
        % elseif dist_to_target < 2.0
        %     fprintf('CONVERGING\n');
        % else
        %     fprintf('NAVIGATING\n');
        % end
    end
    
    % Define objective for current x
    obj = @(U) stage_cost(x,U,N,nu,Ndes,Edes,Ddes, ...
        w_pN,w_vN,w_pE,w_vE,w_pD,w_vD,w_r,w_term,rollout);

    % Angle constraints across horizon
    nonlcon = @(U) nl_constr_angles(x,U,N,nu,rollout,phi_max,theta_max,psi_max);

    % Solve NMPC
    [Uopt,fval,exitflag] = fmincon(obj, U0, [],[],[],[], lbU, ubU, nonlcon, opts);
    if exitflag <= 0
        fprintf('   → Optimization FAILED (exitflag=%d)\n', exitflag);
        Uopt = U0;
    end

    % Store predicted trajectory from the optimal sequence
    [Xpred] = rollout(x, Uopt);
    pred_traj{k} = Xpred;

    % Apply first control
    u0 = Uopt(1:nu);
    x  = f_step(x, u0);

    % Shift warm start
    U0 = [Uopt(nu+1:end); Uopt(end-nu+1:end)]; % shift and repeat last

    % Log
    logU(:,k)   = u0;
    logX(:,k+1) = x;

    % Check convergence in position
    if all(abs(x([1 3 5]) - [Ndes Edes Ddes]') < tol_pos)
        fprintf('\n*** TARGET REACHED! Step %d (%.1fs) ***\n', k, k*dt);
        fprintf('Final error: %.3fm\n', norm(x([1 3 5]) - [Ndes; Edes; Ddes]));
        break;
    end
    k = k + 1;
end

if k > MAX_STEPS
    fprintf('\n*** Max steps reached. Final distance: %.2fm ***\n', dist_to_target);
end

steps_used = min(k, MAX_STEPS);

fprintf('\n=== Simulation complete: %d steps (%.1fs) ===\n', steps_used, steps_used*dt);

% Extract trajectory data for plotting
t = (0:steps_used)*dt;
% UAV trajectory
N_traj = logX(1,1:min(steps_used+1, size(logX,2)));
E_traj = logX(3,1:steps_used+1);
D_traj = logX(5,1:steps_used+1);
phi_traj   = logX(7,1:steps_used+1);
theta_traj = logX(9,1:steps_used+1);
psi_traj   = logX(11,1:steps_used+1);

% Payload trajectory
Np_traj = logX(13,1:steps_used+1);
Ep_traj = logX(15,1:steps_used+1);
Dp_traj = logX(17,1:steps_used+1);

%% ======================= Visualization Section =========================
% %% Plots ------------------------------------------------------------------
figure('Name','Position (NED) - UAV and Payload');
subplot(3,1,1); 
plot(t,N_traj,'b-','LineWidth',1.6); hold on; 
plot(t,Np_traj,'r--','LineWidth',1.6); 
yline(Ndes,'k--'); ylabel('N [m]'); grid on;
legend('UAV','Payload','Target','Location','best');

subplot(3,1,2); 
plot(t,E_traj,'b-','LineWidth',1.6); hold on; 
plot(t,Ep_traj,'r--','LineWidth',1.6); 
yline(Edes,'k--'); ylabel('E [m]'); grid on;
legend('UAV','Payload','Target','Location','best');

subplot(3,1,3); 
plot(t,D_traj,'b-','LineWidth',1.6); hold on; 
plot(t,Dp_traj,'r--','LineWidth',1.6); 
yline(Ddes,'k--'); ylabel('D [m]'); xlabel('time [s]'); grid on;
legend('UAV','Payload','Target','Location','best');

figure('Name','Attitude');
subplot(3,1,1); plot(t,rad2deg(phi_traj),'LineWidth',1.6); ylabel('\phi [deg]'); grid on;
subplot(3,1,2); plot(t,rad2deg(theta_traj),'LineWidth',1.6); ylabel('\theta [deg]'); grid on;
subplot(3,1,3); plot(t,rad2deg(psi_traj),'LineWidth',1.6); ylabel('\psi [deg]'); xlabel('time [s]'); grid on;

figure('Name','Thrust Control');
tt = (0:steps_used-1)*dt;
plot(tt, logU(1,1:steps_used),'b-','LineWidth',2);
ylabel('Thrust [N]'); xlabel('time [s]'); grid on;
legend('Thrust (T)', 'Location', 'best');
title('Thrust Command');

figure('Name','Torque Controls');
tt = (0:steps_used-1)*dt;
subplot(3,1,1);
plot(tt, logU(2,1:steps_used),'r-','LineWidth',1.5);
ylabel('\tau_{\phi} [N⋅m]'); grid on;
legend('Roll Torque', 'Location', 'best');
title('Roll Torque');

subplot(3,1,2);
plot(tt, logU(3,1:steps_used),'g-','LineWidth',1.5);
ylabel('\tau_{\theta} [N⋅m]'); grid on;
legend('Pitch Torque', 'Location', 'best');
title('Pitch Torque');

subplot(3,1,3);
plot(tt, logU(4,1:steps_used),'b-','LineWidth',1.5);
ylabel('\tau_{\psi} [N⋅m]'); xlabel('time [s]'); grid on;
legend('Yaw Torque', 'Location', 'best');
title('Yaw Torque');
% Ensure this figure stays focused
sgtitle('UAV Control Torques'); % Add overall figure title

% 3D path
figure('Name','3D Path (NED) - UAV and Payload');
plot3(N_traj, E_traj, D_traj, 'b-', 'LineWidth', 2); grid on; hold on;
plot3(Np_traj, Ep_traj, Dp_traj, 'r--', 'LineWidth', 2);
plot3(Ndes, Edes, Ddes, 'go', 'MarkerFaceColor','g','MarkerSize',8); 
% Show tether connections at a few points
for i = 1:10:length(t)
    plot3([N_traj(i) Np_traj(i)], [E_traj(i) Ep_traj(i)], [D_traj(i) Dp_traj(i)], 'k-', 'LineWidth', 0.5);
end
% Fix NED frame: In NED, positive D is downward, so reverse Z-axis
set(gca, 'ZDir', 'reverse');
xlabel('North [m]'); ylabel('East [m]'); zlabel('Down [m]');
legend('UAV Path','Payload Path','Target','Tethers','Location','best');
title('UAV and Payload Trajectories (NED Frame)');

%% Animation of UAV and Payload
% Create animation figure
f_anim = figure('Name', 'UAV-Payload Animation', 'Position', [100, 100, 800, 600], 'Color', 'w');
ax_anim = axes(f_anim, 'Units', 'pixels', 'Position', [50, 50, 700, 500]);
hold on; grid on; axis equal;

% Set axis limits with some padding around both trajectories
all_N = [N_traj, Np_traj];
all_E = [E_traj, Ep_traj];
all_D = [D_traj, Dp_traj];

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

% Create target marker
target_marker = plot3(Ndes, Edes, Ddes, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
text(Ndes, Edes, Ddes, '  Target', 'FontSize', 12);

% Create trajectory lines (will update during animation)
uav_traj_line = plot3(N_traj(1), E_traj(1), D_traj(1), 'b-', 'LineWidth', 1.5);
payload_traj_line = plot3(Np_traj(1), Ep_traj(1), Dp_traj(1), 'r--', 'LineWidth', 1.5);

% Create drone-like UAV structure instead of simple markers
drone_size = 0.3;  % Size of the drone
arm_length = drone_size * 0.8;
prop_radius = drone_size * 0.4;

% Drone body (center cross structure)
uav_body_x = plot3([N_traj(1)-arm_length N_traj(1)+arm_length], [E_traj(1) E_traj(1)], [D_traj(1) D_traj(1)], 'k-', 'LineWidth', 3);
uav_body_y = plot3([N_traj(1) N_traj(1)], [E_traj(1)-arm_length E_traj(1)+arm_length], [D_traj(1) D_traj(1)], 'k-', 'LineWidth', 3);

% Propellers (4 circles at arm ends)
theta_prop = linspace(0, 2*pi, 16);
prop_x = prop_radius * cos(theta_prop);
prop_y = prop_radius * sin(theta_prop);
prop_z = zeros(size(prop_x));

% Initialize propellers array
propellers = gobjects(4,1);

% Front propeller (North+)
propellers(1) = plot3(N_traj(1) + arm_length + prop_x, E_traj(1) + prop_y, D_traj(1) + prop_z, 'b-', 'LineWidth', 2);
% Back propeller (North-)
propellers(2) = plot3(N_traj(1) - arm_length + prop_x, E_traj(1) + prop_y, D_traj(1) + prop_z, 'b-', 'LineWidth', 2);
% Right propeller (East+)
propellers(3) = plot3(N_traj(1) + prop_y, E_traj(1) + arm_length + prop_x, D_traj(1) + prop_z, 'b-', 'LineWidth', 2);
% Left propeller (East-)
propellers(4) = plot3(N_traj(1) + prop_y, E_traj(1) - arm_length + prop_x, D_traj(1) + prop_z, 'b-', 'LineWidth', 2);

% Payload marker (keep simple)
payload_marker = plot3(Np_traj(1), Ep_traj(1), Dp_traj(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Create tether line
tether_line = plot3([N_traj(1) Np_traj(1)], [E_traj(1) Ep_traj(1)], [D_traj(1) Dp_traj(1)], 'k-', 'LineWidth', 2);

% Create UAV orientation lines (Roll, Pitch, Yaw axes)
uav_size = 0.5;
roll_line = quiver3(N_traj(1), E_traj(1), D_traj(1), 0, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.3);
pitch_line = quiver3(N_traj(1), E_traj(1), D_traj(1), 0, 0, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.3);
yaw_line = quiver3(N_traj(1), E_traj(1), D_traj(1), 0, 0, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Add legend
legend([uav_body_x, payload_marker, tether_line, target_marker, roll_line, pitch_line, yaw_line], ...
       'Drone', 'Payload', 'Tether', 'Target', 'Roll', 'Pitch', 'Yaw', ...
       'Location', 'best');

% Create video writer
videoName = 'uav_payload_animation.mp4';
v = VideoWriter(videoName, 'MPEG-4');
v.FrameRate = 20;  % Adjust speed (frames per second)
open(v);

% Animation loop
for i = 1:length(t)
    % Update trajectory lines
    set(uav_traj_line, 'XData', N_traj(1:i), 'YData', E_traj(1:i), 'ZData', D_traj(1:i));
    set(payload_traj_line, 'XData', Np_traj(1:i), 'YData', Ep_traj(1:i), 'ZData', Dp_traj(1:i));
    
    % Update drone body position and orientation
    current_pos = [N_traj(i), E_traj(i), D_traj(i)];
    current_att = [phi_traj(i), theta_traj(i), psi_traj(i)]; % [roll, pitch, yaw]
    
    % Rotation matrix for drone orientation
    R = angle2dcm(current_att(3), current_att(2), current_att(1), 'ZYX');
    
    % Update drone body arms
    arm_length = 0.24; % 0.8 * 0.3
    arm_x_local = [-arm_length, arm_length, 0, 0];
    arm_y_local = [0, 0, -arm_length, arm_length];
    arm_z_local = [0, 0, 0, 0];
    
    % Rotate arms based on drone attitude
    arm_points = R * [arm_x_local; arm_y_local; arm_z_local];
    set(uav_body_x, 'XData', [current_pos(1) + arm_points(1,1), current_pos(1) + arm_points(1,2)], ...
                    'YData', [current_pos(2) + arm_points(2,1), current_pos(2) + arm_points(2,2)], ...
                    'ZData', [current_pos(3) + arm_points(3,1), current_pos(3) + arm_points(3,2)]);
    set(uav_body_y, 'XData', [current_pos(1) + arm_points(1,3), current_pos(1) + arm_points(1,4)], ...
                    'YData', [current_pos(2) + arm_points(2,3), current_pos(2) + arm_points(2,4)], ...
                    'ZData', [current_pos(3) + arm_points(3,3), current_pos(3) + arm_points(3,4)]);
    
    % Update propellers at arm ends
    prop_radius = 0.12; % 0.4 * 0.3
    theta_prop = linspace(0, 2*pi, 16);
    prop_x_circle = prop_radius * cos(theta_prop);
    prop_y_circle = prop_radius * sin(theta_prop);
    prop_z_circle = zeros(size(prop_x_circle));
    
    % Propeller positions in body frame
    prop_positions_local = [arm_length, -arm_length, 0, 0; 
                           0, 0, arm_length, -arm_length; 
                           0, 0, 0, 0];
    prop_positions_global = R * prop_positions_local;
    
    for j = 1:4
        prop_center = current_pos + prop_positions_global(:,j)';
        % Create propeller circle in local frame then transform
        prop_circle_local = [prop_x_circle; prop_y_circle; prop_z_circle];
        prop_circle_global = R * prop_circle_local;
        
        set(propellers(j), 'XData', prop_center(1) + prop_circle_global(1,:), ...
                          'YData', prop_center(2) + prop_circle_global(2,:), ...
                          'ZData', prop_center(3) + prop_circle_global(3,:));
    end
    
    % Update payload position
    set(payload_marker, 'XData', Np_traj(i), 'YData', Ep_traj(i), 'ZData', Dp_traj(i));
    
    % Update tether line
    set(tether_line, 'XData', [N_traj(i) Np_traj(i)], 'YData', [E_traj(i) Ep_traj(i)], 'ZData', [D_traj(i) Dp_traj(i)]);
    
    % Update UAV orientation arrows (Roll, Pitch, Yaw) - move with drone
    % Direction vectors in body frame
    axis_length = 0.5;
    x_body = axis_length * [1; 0; 0];  % Roll axis (red)
    y_body = axis_length * [0; 1; 0];  % Pitch axis (green)  
    z_body = axis_length * [0; 0; 1];  % Yaw axis (blue)
    
    % Transform to NED frame
    x_ned = R * x_body;
    y_ned = R * y_body;
    z_ned = R * z_body;
    
    % Update orientation arrows to follow the drone
    set(roll_line, 'XData', current_pos(1), 'YData', current_pos(2), 'ZData', current_pos(3), ...
                  'UData', x_ned(1), 'VData', x_ned(2), 'WData', x_ned(3));
    set(pitch_line, 'XData', current_pos(1), 'YData', current_pos(2), 'ZData', current_pos(3), ...
                   'UData', y_ned(1), 'VData', y_ned(2), 'WData', y_ned(3));
    set(yaw_line, 'XData', current_pos(1), 'YData', current_pos(2), 'ZData', current_pos(3), ...
                 'UData', z_ned(1), 'VData', z_ned(2), 'WData', z_ned(3));
    
    % Add time label and tether length info
    tether_length = norm([N_traj(i) - Np_traj(i); E_traj(i) - Ep_traj(i); D_traj(i) - Dp_traj(i)]);
    time_text = sprintf('Time: %.2f s, Tether Length: %.2f m', t(i), tether_length);
    title(['UAV-Payload Animation (NED) - ', time_text]);
    
    % Capture frame
    drawnow;
    frame = getframe(ax_anim);
    writeVideo(v, frame);
end

% Close video file
close(v);
fprintf('Animation saved: %s\n', videoName);

fprintf('\n=== MISSION COMPLETE ===\n');
