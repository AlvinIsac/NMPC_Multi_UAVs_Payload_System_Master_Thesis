function plot_mpc_results(out)
% Plot MPC simulation results with animation
% Inputs:
%   out: Simulink SimulationOutput structure containing:
%        - states_out: State data [30 x 1 x Nt] or similar format
%        - controls_out: Control data [8*Nt x 1] or similar format  
%        - tout: Time vector [Nt x 1]
%
% State vector (30 elements):
%   UAV1: [N1, vN1, E1, vE1, D1, vD1, phi1, p1, theta1, q1, psi1, r1] (1-12)
%   UAV2: [N2, vN2, E2, vE2, D2, vD2, phi2, p2, theta2, q2, psi2, r2] (13-24)
%   Payload: [Np, vNp, Ep, vEp, Dp, vDp] (25-30)
%
% Control vector (8 elements):
%   [T1, tau_phi1, tau_theta1, tau_psi1, T2, tau_phi2, tau_theta2, tau_psi2]

% ----------------- extract and reshape data -----------------
[t_states, X] = extract_matrix(out.states_out, 30, out.tout);   % Nt x 30
[t_controls, U] = extract_matrix(out.controls_out, 8, out.tout); % NtU x 8

% Extract dimensions
steps_used = size(X, 1) - 1;

% Extract trajectory data
% UAV1 trajectory
N1_traj = X(:, 1);
E1_traj = X(:, 3);
D1_traj = X(:, 5);
vN1_traj = X(:, 2);
vE1_traj = X(:, 4);
vD1_traj = X(:, 6);
phi1_traj = X(:, 7);
theta1_traj = X(:, 9);
psi1_traj = X(:, 11);

% UAV2 trajectory
N2_traj = X(:, 13);
E2_traj = X(:, 15);
D2_traj = X(:, 17);
vN2_traj = X(:, 14);
vE2_traj = X(:, 16);
vD2_traj = X(:, 18);
phi2_traj = X(:, 19);
theta2_traj = X(:, 21);
psi2_traj = X(:, 23);

% Payload trajectory
Np_traj = X(:, 25);
Ep_traj = X(:, 27);
Dp_traj = X(:, 29);
vNp_traj = X(:, 26);
vEp_traj = X(:, 28);
vDp_traj = X(:, 30);

%% ======================= Position Plots =========================
figure('Name','Position (NED) - UAVs and Payload', 'Color', 'w');
subplot(3,1,1);
plot(t_states, N1_traj, 'b-', 'LineWidth', 1.6); hold on;
plot(t_states, N2_traj, 'g-', 'LineWidth', 1.6);
plot(t_states, Np_traj, 'r--', 'LineWidth', 1.6);
ylabel('N [m]'); grid on;
legend('UAV1','UAV2','Payload','Location','best');
title('North Position');

subplot(3,1,2);
plot(t_states, E1_traj, 'b-', 'LineWidth', 1.6); hold on;
plot(t_states, E2_traj, 'g-', 'LineWidth', 1.6);
plot(t_states, Ep_traj, 'r--', 'LineWidth', 1.6);
ylabel('E [m]'); grid on;
legend('UAV1','UAV2','Payload','Location','best');
title('East Position');

subplot(3,1,3);
plot(t_states, D1_traj, 'b-', 'LineWidth', 1.6); hold on;
plot(t_states, D2_traj, 'g-', 'LineWidth', 1.6);
plot(t_states, Dp_traj, 'r--', 'LineWidth', 1.6);
ylabel('D [m]'); xlabel('time [s]'); grid on;
legend('UAV1','UAV2','Payload','Location','best');
title('Down Position');

%% ======================= Velocity Plots =========================
figure('Name','Velocities - UAVs and Payload', 'Color', 'w');
subplot(3,1,1);
plot(t_states, vN1_traj, 'b-', 'LineWidth', 1.6); hold on;
plot(t_states, vN2_traj, 'g-', 'LineWidth', 1.6);
plot(t_states, vNp_traj, 'r--', 'LineWidth', 1.6);
ylabel('vN [m/s]'); grid on;
legend('UAV1','UAV2','Payload','Location','best');
title('North Velocity');

subplot(3,1,2);
plot(t_states, vE1_traj, 'b-', 'LineWidth', 1.6); hold on;
plot(t_states, vE2_traj, 'g-', 'LineWidth', 1.6);
plot(t_states, vEp_traj, 'r--', 'LineWidth', 1.6);
ylabel('vE [m/s]'); grid on;
legend('UAV1','UAV2','Payload','Location','best');
title('East Velocity');

subplot(3,1,3);
plot(t_states, vD1_traj, 'b-', 'LineWidth', 1.6); hold on;
plot(t_states, vD2_traj, 'g-', 'LineWidth', 1.6);
plot(t_states, vDp_traj, 'r--', 'LineWidth', 1.6);
ylabel('vD [m/s]'); xlabel('time [s]'); grid on;
legend('UAV1','UAV2','Payload','Location','best');
title('Down Velocity');

%% ======================= Payload Velocity Magnitude =========================
figure('Name','Payload Velocity Magnitude', 'Color', 'w');
payload_vel_mag = sqrt(vNp_traj.^2 + vEp_traj.^2 + vDp_traj.^2);
plot(t_states, payload_vel_mag, 'r-', 'LineWidth', 2);
ylabel('|v_{payload}| [m/s]'); xlabel('time [s]'); grid on;
title('Total Payload Velocity Magnitude');

%% ======================= Attitude Plots =========================
figure('Name','Attitude (UAV1 & UAV2)', 'Color', 'w');
subplot(3,1,1);
plot(t_states, rad2deg(phi1_traj), 'b-', 'LineWidth', 1.6); hold on;
plot(t_states, rad2deg(phi2_traj), 'g-', 'LineWidth', 1.2);
ylabel('\phi [deg]'); grid on;
legend('UAV1','UAV2','Location','best');
title('Roll Angle');

subplot(3,1,2);
plot(t_states, rad2deg(theta1_traj), 'b-', 'LineWidth', 1.6); hold on;
plot(t_states, rad2deg(theta2_traj), 'g-', 'LineWidth', 1.2);
ylabel('\theta [deg]'); grid on;
legend('UAV1','UAV2','Location','best');
title('Pitch Angle');

subplot(3,1,3);
plot(t_states, rad2deg(psi1_traj), 'b-', 'LineWidth', 1.6); hold on;
plot(t_states, rad2deg(psi2_traj), 'g-', 'LineWidth', 1.2);
ylabel('\psi [deg]'); xlabel('time [s]'); grid on;
legend('UAV1','UAV2','Location','best');
title('Yaw Angle');

%% ======================= Control Plots =========================
figure('Name','Thrust Controls', 'Color', 'w');
subplot(2,1,1);
plot(t_controls, U(:, 1), 'b-', 'LineWidth', 2);
ylabel('Thrust [N]'); grid on;
title('UAV1 Thrust Command');

subplot(2,1,2);
plot(t_controls, U(:, 5), 'g-', 'LineWidth', 2);
ylabel('Thrust [N]'); xlabel('time [s]'); grid on;
title('UAV2 Thrust Command');

figure('Name','Torque Controls', 'Color', 'w');
subplot(3,1,1);
plot(t_controls, U(:, 2), 'b-', 'LineWidth', 1.5); hold on;
plot(t_controls, U(:, 6), 'g-', 'LineWidth', 1.5);
ylabel('\tau_{\phi} [N⋅m]'); grid on;
legend('UAV1 Roll Torque', 'UAV2 Roll Torque', 'Location', 'best');
title('Roll Torques');

subplot(3,1,2);
plot(t_controls, U(:, 3), 'b-', 'LineWidth', 1.5); hold on;
plot(t_controls, U(:, 7), 'g-', 'LineWidth', 1.5);
ylabel('\tau_{\theta} [N⋅m]'); grid on;
legend('UAV1 Pitch Torque', 'UAV2 Pitch Torque', 'Location', 'best');
title('Pitch Torques');

subplot(3,1,3);
plot(t_controls, U(:, 4), 'b-', 'LineWidth', 1.5); hold on;
plot(t_controls, U(:, 8), 'g-', 'LineWidth', 1.5);
ylabel('\tau_{\psi} [N⋅m]'); xlabel('time [s]'); grid on;
legend('UAV1 Yaw Torque', 'UAV2 Yaw Torque', 'Location', 'best');
title('Yaw Torques');

fprintf('\n=== PLOTTING COMPLETE ===\n');
tileFigs(2,3)
end
function [t, M] = extract_matrix(A, nSignals, tDefault)
% Convert a variety of array shapes into [Nt x nSignals] with a time vector.
%
% Supported input shapes:
%   [n x 1 x Nt]  (your states)  -> permuted
%   [n x Nt]      -> transposed
%   [Nt x n]      -> as-is
%   [n*Nt x 1]    (your controls) -> reshaped
%
% Time:
%   If no time is present, we use tDefault. If we detect stacked-column
%   controls with NtU samples, we linearly space t over [tDefault(1) tDefault(end)].

    % ---- reshape data to 2D [Nt x nSignals] ----
    sz = size(A);

    if isvector(A) && numel(A) ~= nSignals
        % Stacked column: [n*Nt x 1]
        L = numel(A);
        assert(mod(L,nSignals)==0, 'Length %d not divisible by nSignals=%d.', L, nSignals);
        Nt = L / nSignals;
        M  = reshape(A, [nSignals Nt]).';      % -> [Nt x n]
        t  = linspace(tDefault(1), tDefault(end), Nt).';  % build control time

    else
        B = squeeze(A);                         % remove singleton dims
        if size(B,1) == nSignals && size(B,2) > 1
            % [n x Nt]
            M = B.';                            % -> [Nt x n]
        elseif size(B,2) == nSignals
            % [Nt x n]
            M = B;
        elseif ndims(A) == 3 && sz(1)==nSignals && sz(2)==1
            % [n x 1 x Nt]
            M = permute(A,[3 1 2]);             % -> [Nt x n x 1]
            M = reshape(M, sz(3), nSignals);
        else
            error('Unexpected data shape for signal (%s).', mat2str(sz));
        end
        % use provided default time (e.g., out.tout)
        Nt = size(M,1);
        if numel(tDefault)==Nt
            t = tDefault(:);
        else
            % Fallback: build a uniform time vector across the same span
            t = linspace(tDefault(1), tDefault(end), Nt).';
        end
    end
end


function tileFigs(rows, cols, varargin)
% tileFigs(rows, cols, 'Padding',8, 'Monitor',1, 'Margins',[10 40 10 50])
% Places all open MATLAB figure windows in a rows×cols grid with **no overlap**.
% Uses the figure **OuterPosition** (includes title bar + borders).

% ---- options ----
p = inputParser;
addParameter(p,'Padding',6);                 % space between cells (px)
addParameter(p,'Monitor',1);                 % 1 = primary
addParameter(p,'Margins',[8 35 8 45]);       % [left bottom right top] px
parse(p,varargin{:});
pad = p.Results.Padding;
mIdx = p.Results.Monitor;
marg = p.Results.Margins;

% ---- figures (sorted by figure number) ----
figs = findall(0,'Type','figure','Visible','on');
if isempty(figs), return; end
[~,ix] = sort([figs.Number]);
figs = figs(ix);

% ---- monitor bounds ----
mp = get(0,'MonitorPositions');  % [x y w h] per monitor
mIdx = min(mIdx, size(mp,1));
mon = mp(mIdx,:);

% usable area (account for menu bar/dock with margins)
usableW = mon(3) - (marg(1) + marg(3));
usableH = mon(4) - (marg(2) + marg(4));

% cell size (exact integers; subtract inter-cell padding)
cellW = floor((usableW - pad*(cols-1)) / cols);
cellH = floor((usableH - pad*(rows-1)) / rows);

% ---- place each figure ----
n = numel(figs);
for k = 1:n
    r = floor((k-1)/cols);         % 0-based row, top→bottom
    c = mod(k-1, cols);            % 0-based col, left→right

    % y from bottom; flip rows to start at the top
    x = mon(1) + marg(1) + c*(cellW + pad);
    y = mon(2) + marg(2) + (rows-1-r)*(cellH + pad);

    set(figs(k), 'Units','pixels', 'WindowStyle','normal');
    % Use OuterPosition to include window chrome → prevents overlap
    set(figs(k), 'OuterPosition', [x, y, cellW, cellH]);
end
end