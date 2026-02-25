function xdot = continuous_dynamics_2uav(x, u)
    
    % Track function call count
    persistent continuous_dynamics_call_count
    if isempty(continuous_dynamics_call_count)
        continuous_dynamics_call_count = 0;
    end
    continuous_dynamics_call_count = continuous_dynamics_call_count + 1;
    
    %% Physical parameters
    m   = 1.5;               
    mP  = 1;                 
    g   = 9.80665;           
    Ixx = 0.022; Iyy = 0.022; Izz = 0.04;  
    
    
    L   = 1.6;               
    k   = 80;                
    b   = 2.0;               
    
    %% Extract states
    % UAV1 states (indices 1-12)
    x1 = x(1:12);
    N1 = x1(1); vN1 = x1(2); E1 = x1(3); vE1 = x1(4); D1 = x1(5); vD1 = x1(6);
    phi1 = x1(7); p1 = x1(8); th1 = x1(9); q1 = x1(10); psi1 = x1(11); r1 = x1(12);

    % UAV2 states (indices 13-24)
    x2 = x(13:24);
    N2 = x2(1); vN2 = x2(2); E2 = x2(3); vE2 = x2(4); D2 = x2(5); vD2 = x2(6);
    phi2 = x2(7); p2 = x2(8); th2 = x2(9); q2 = x2(10); psi2 = x2(11); r2 = x2(12);

    % Payload states (indices 25-30)
    xp = x(25:30);
    Np = xp(1); vNp = xp(2); Ep = xp(3); vEp = xp(4); Dp = xp(5); vDp = xp(6);

    %% Extract controls
    u1 = u(1:4); u2 = u(5:8);
    T1 = u1(1); tau_phi1 = u1(2); tau_theta1 = u1(3); tau_psi1 = u1(4);
    T2 = u2(1); tau_phi2 = u2(2); tau_theta2 = u2(3); tau_psi2 = u2(4);

    %% Calculate tether forces
    % Position vectors
    p1_vec = [N1; E1; D1]; v1_vec = [vN1; vE1; vD1];
    p2_vec = [N2; E2; D2]; v2_vec = [vN2; vE2; vD2];
    pp_vec = [Np; Ep; Dp]; vp_vec = [vNp; vEp; vDp];

    % Tether force from UAV1 to payload
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

    % Tether force from UAV2 to payload
    r2p = pp_vec - p2_vec; d2 = norm(r2p);
    if (d2 - L) > 0
        u2t = r2p / d2;
        F2_spring = k*(d2 - L)*u2t;
        vel_rel2 = vp_vec - v2_vec;
        F2_damping = b * vel_rel2;
        F2 = F2_spring + F2_damping; % force on payload from UAV2
    else
        F2 = [0;0;0];
    end

    %% UAV1 dynamics
    aN1 = - (T1/m)*( sin(phi1)*sin(psi1) + cos(phi1)*sin(th1)*cos(psi1) ) + F1(1)/m;
    aE1 = - (T1/m)*( cos(phi1)*sin(th1)*sin(psi1) - sin(phi1)*cos(psi1) ) + F1(2)/m;
    aD1 = - (T1/m)*( cos(phi1)*cos(th1) ) + g + F1(3)/m;

    % Rotational accelerations
    p1_dot = (tau_phi1   - (Iyy - Izz)*q1*r1)/Ixx;
    q1_dot = (tau_theta1 - (Izz - Ixx)*p1*r1)/Iyy;
    r1_dot = (tau_psi1   - (Ixx - Iyy)*p1*q1)/Izz;

    % Euler angle rates
    cth1 = cos(th1);  if abs(cth1) < 1e-6, cth1 = sign(cth1)*1e-6; end
    Winv1 = [ 1,  sin(phi1)*tan(th1),  cos(phi1)*tan(th1);
              0,  cos(phi1),          -sin(phi1);
              0,  sin(phi1)/cth1,      cos(phi1)/cth1 ];
    eul1_dot = Winv1 * [p1; q1; r1];

    % UAV1 state derivatives
    x1_dot = [vN1; aN1; vE1; aE1; vD1; aD1; eul1_dot(1); p1_dot; eul1_dot(2); q1_dot; eul1_dot(3); r1_dot];

    %% UAV2 dynamics
    % Translational accelerations (NED frame)
    aN2 = - (T2/m)*( sin(phi2)*sin(psi2) + cos(phi2)*sin(th2)*cos(psi2) ) + F2(1)/m;
    aE2 = - (T2/m)*( cos(phi2)*sin(th2)*sin(psi2) - sin(phi2)*cos(psi2) ) + F2(2)/m;
    aD2 = - (T2/m)*( cos(phi2)*cos(th2) ) + g + F2(3)/m;

    % Rotational accelerations
    p2_dot = (tau_phi2   - (Iyy - Izz)*q2*r2)/Ixx;
    q2_dot = (tau_theta2 - (Izz - Ixx)*p2*r2)/Iyy;
    r2_dot = (tau_psi2   - (Ixx - Iyy)*p2*q2)/Izz;

    % Euler angle rates
    cth2 = cos(th2);  if abs(cth2) < 1e-6, cth2 = sign(cth2)*1e-6; end
    Winv2 = [ 1,  sin(phi2)*tan(th2),  cos(phi2)*tan(th2);
              0,  cos(phi2),          -sin(phi2);
              0,  sin(phi2)/cth2,      cos(phi2)/cth2 ];
    eul2_dot = Winv2 * [p2; q2; r2];

    % UAV2 state derivatives
    x2_dot = [vN2; aN2; vE2; aE2; vD2; aD2; eul2_dot(1); p2_dot; eul2_dot(2); q2_dot; eul2_dot(3); r2_dot];

    %% Payload dynamics
    % Payload accelerations (tether forces + gravity)
    aNp = - (F1(1) + F2(1)) / mP;
    aEp = - (F1(2) + F2(2)) / mP;
    aDp = - (F1(3) + F2(3)) / mP + g;

    % Payload state derivatives
    xp_dot = [vNp; aNp; vEp; aEp; vDp; aDp];

    %% Combine all derivatives
    xdot = [x1_dot; x2_dot; xp_dot];
    
    % Export call count to workspace when simulation ends
    assignin('base', 'continuous_dynamics_call_count', continuous_dynamics_call_count);
end
