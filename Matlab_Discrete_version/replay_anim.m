function replay_anim()
%REPLAY_ANIM Replay the UAV-Payload animation (full detailed version)
%   This function replays the animation using the trajectory data
%   from the workspace. Make sure to run uav_mpc_target.m first.

    % Check if animation data exists in workspace
    if ~evalin('base', 'exist(''t'', ''var'')') || ~evalin('base', 'exist(''N1_traj'', ''var'')')
        fprintf('Animation data not found! Please run uav_mpc_target.m first.\n');
        return;
    end
    
    % Get data from base workspace
    t = evalin('base', 't');
    N1_traj = evalin('base', 'N1_traj');
    E1_traj = evalin('base', 'E1_traj');
    D1_traj = evalin('base', 'D1_traj');
    N2_traj = evalin('base', 'N2_traj');
    E2_traj = evalin('base', 'E2_traj');
    D2_traj = evalin('base', 'D2_traj');
    Np_traj = evalin('base', 'Np_traj');
    Ep_traj = evalin('base', 'Ep_traj');
    Dp_traj = evalin('base', 'Dp_traj');
    
    % Check if target variables exist (optional)
    has_target = false;
    try
        Ndes = evalin('base', 'Ndes');
        Edes = evalin('base', 'Edes');
        Ddes = evalin('base', 'Ddes');
        has_target = true;
        fprintf('Target data found - including target visualization.\n');
    catch
        fprintf('Target data not available - running without target visualization.\n');
    end
    
    % Get attitude data
    phi1_traj = evalin('base', 'phi1_traj');
    theta1_traj = evalin('base', 'theta1_traj');
    psi1_traj = evalin('base', 'psi1_traj');
    phi2_traj = evalin('base', 'phi2_traj');
    theta2_traj = evalin('base', 'theta2_traj');
    psi2_traj = evalin('base', 'psi2_traj');
    
    fprintf('Replaying detailed animation... (Close figure to stop)\n');
    
    % Create animation figure - same as original
    f_anim = figure('Name', 'UAV-Payload Animation Replay', 'Position', [100, 100, 800, 600], 'Color', 'w');
    ax_anim = axes(f_anim, 'Units', 'pixels', 'Position', [50, 50, 700, 500]);
    hold on; grid on; axis equal;

    % Set axis limits with some padding around both trajectories
    if has_target
        all_N = [N1_traj, N2_traj, Np_traj, Ndes];
        all_E = [E1_traj, E2_traj, Ep_traj, Edes];
        all_D = [D1_traj, D2_traj, Dp_traj, Ddes];
    else
        all_N = [N1_traj, N2_traj, Np_traj];
        all_E = [E1_traj, E2_traj, Ep_traj];
        all_D = [D1_traj, D2_traj, Dp_traj];
    end

    N_pad = max(abs(max(all_N) - min(all_N)) * 0.2, 2);
    E_pad = max(abs(max(all_E) - min(all_E)) * 0.2, 2);
    D_pad = max(abs(max(all_D) - min(all_D)) * 0.2, 2);

    xlim([min(all_N)-N_pad, max(all_N)+N_pad]);
    ylim([min(all_E)-E_pad, max(all_E)+E_pad]);
    zlim([min(all_D)-D_pad, max(all_D)+D_pad]);

    % In NED, positive D is downward
    set(ax_anim, 'ZDir', 'reverse');
    xlabel('North [m]'); ylabel('East [m]'); zlabel('Down [m]');
    title('UAV-Payload Animation Replay (NED)');

    % Add target position marker (if available)
    if has_target
        target_marker = plot3(Ndes, Edes, Ddes, 'mx', 'MarkerSize', 15, 'LineWidth', 4);
    end

    % Create trajectory lines (will update during animation)
    traj1_line = plot3(N1_traj(1), E1_traj(1), D1_traj(1), 'b-', 'LineWidth', 1.5);
    traj2_line = plot3(N2_traj(1), E2_traj(1), D2_traj(1), 'g-', 'LineWidth', 1.5);
    trajp_line = plot3(Np_traj(1), Ep_traj(1), Dp_traj(1), 'r--', 'LineWidth', 1.5);

    % Add formation center trajectory
    center_N_traj = (N1_traj + N2_traj) / 2;
    center_E_traj = (E1_traj + E2_traj) / 2;
    center_D_traj = (D1_traj + D2_traj) / 2;
    center_traj_line = plot3(center_N_traj(1), center_E_traj(1), center_D_traj(1), 'k:', 'LineWidth', 2);

    % Create drone-like UAV structures
    drone_size = 0.3;
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
    propellers1 = gobjects(4,1);
    propellers2 = gobjects(4,1);

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

    % Payload as solid cubic box instead of simple marker
    box_size = 0.2;  % Size of the cubic box
    % Define the 8 vertices of a cube centered at origin
    vertices = [
        -1, -1, -1;  1, -1, -1;  1,  1, -1; -1,  1, -1;  % bottom face
        -1, -1,  1;  1, -1,  1;  1,  1,  1; -1,  1,  1   % top face
    ] * box_size/2;

    % Define the 6 faces of the cube (each face has 4 vertices)
    faces = [
        1, 2, 3, 4;  % bottom face (z = -box_size/2)
        5, 8, 7, 6;  % top face (z = +box_size/2)
        1, 5, 6, 2;  % front face (y = -box_size/2)
        4, 3, 7, 8;  % back face (y = +box_size/2)
        1, 4, 8, 5;  % left face (x = -box_size/2)
        2, 6, 7, 3   % right face (x = +box_size/2)
    ];

    % Initialize payload box faces as patch objects (solid faces)
    payload_box_faces = gobjects(6,1);
    for j = 1:6
        face_vertices = vertices(faces(j,:),:) + [Np_traj(1), Ep_traj(1), Dp_traj(1)];
        payload_box_faces(j) = patch('XData', face_vertices(:,1), 'YData', face_vertices(:,2), 'ZData', face_vertices(:,3), ...
                                     'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', 1.0, 'LineWidth', 1);
    end

    % Tether lines
    tether1_line = plot3([N1_traj(1) Np_traj(1)], [E1_traj(1) Ep_traj(1)], [D1_traj(1) Dp_traj(1)], 'k-', 'LineWidth', 2);
    tether2_line = plot3([N2_traj(1) Np_traj(1)], [E2_traj(1) Ep_traj(1)], [D2_traj(1) Dp_traj(1)], 'k-', 'LineWidth', 2);

    % Create UAV orientation lines
    uav_size = 0.5;
    roll1_line = quiver3(N1_traj(1), E1_traj(1), D1_traj(1), 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    pitch1_line = quiver3(N1_traj(1), E1_traj(1), D1_traj(1), 0, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    yaw1_line = quiver3(N1_traj(1), E1_traj(1), D1_traj(1), 0, 0, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    roll2_line = quiver3(N2_traj(1), E2_traj(1), D2_traj(1), 0, 0, 0, 'r--', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    pitch2_line = quiver3(N2_traj(1), E2_traj(1), D2_traj(1), 0, 0, 0, 'g--', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    yaw2_line = quiver3(N2_traj(1), E2_traj(1), D2_traj(1), 0, 0, 0, 'b--', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

    % Add legend
    if has_target
        legend([uav1_body_x, uav2_body_x, payload_box_faces(1), tether1_line, tether2_line, target_marker, center_traj_line, ...
                roll1_line, pitch1_line, yaw1_line, roll2_line, pitch2_line, yaw2_line], ...
            'UAV1', 'UAV2', 'Payload', 'Tether1', 'Tether2', 'Target', 'Center Path', ...
            'UAV1 X-axis (Roll)', 'UAV1 Y-axis (Pitch)', 'UAV1 Z-axis (Yaw)', ...
            'UAV2 X-axis (Roll)', 'UAV2 Y-axis (Pitch)', 'UAV2 Z-axis (Yaw)', ...
            'Location', 'best');
    else
        legend([uav1_body_x, uav2_body_x, payload_box_faces(1), tether1_line, tether2_line, center_traj_line, ...
                roll1_line, pitch1_line, yaw1_line, roll2_line, pitch2_line, yaw2_line], ...
            'UAV1', 'UAV2', 'Payload', 'Tether1', 'Tether2', 'Center Path', ...
            'UAV1 X-axis (Roll)', 'UAV1 Y-axis (Pitch)', 'UAV1 Z-axis (Yaw)', ...
            'UAV2 X-axis (Roll)', 'UAV2 Y-axis (Pitch)', 'UAV2 Z-axis (Yaw)', ...
            'Location', 'best');
    end

    % Animation loop - SAME SPEED AS ORIGINAL
    for i = 1:length(t)
        if ~ishandle(f_anim), break; end  % Stop if figure is closed
        
        % Update trajectory lines
        set(traj1_line, 'XData', N1_traj(1:i), 'YData', E1_traj(1:i), 'ZData', D1_traj(1:i));
        set(traj2_line, 'XData', N2_traj(1:i), 'YData', E2_traj(1:i), 'ZData', D2_traj(1:i));
        set(trajp_line, 'XData', Np_traj(1:i), 'YData', Ep_traj(1:i), 'ZData', Dp_traj(1:i));
        
        % Update center trajectory
        set(center_traj_line, 'XData', center_N_traj(1:i), 'YData', center_E_traj(1:i), 'ZData', center_D_traj(1:i));
        
        % Update UAV1 drone body position and orientation
        current_pos1 = [N1_traj(i), E1_traj(i), D1_traj(i)];
        current_att1 = [phi1_traj(i), theta1_traj(i), psi1_traj(i)];
        
        % Rotation matrix for UAV1
        R1 = angle2dcm(current_att1(3), current_att1(2), current_att1(1), 'ZYX');
        
        % Update UAV1 drone body arms
        arm_length = 0.24;
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
        current_att2 = [phi2_traj(i), theta2_traj(i), psi2_traj(i)];
        
        % Rotation matrix for UAV2
        R2 = angle2dcm(current_att2(3), current_att2(2), current_att2(1), 'ZYX');
        
        % Rotate arms based on UAV2 attitude
        arm_points2 = R2 * [arm_x_local; arm_y_local; arm_z_local];
        set(uav2_body_x, 'XData', [current_pos2(1) + arm_points2(1,1), current_pos2(1) + arm_points2(1,2)], ...
                         'YData', [current_pos2(2) + arm_points2(2,1), current_pos2(2) + arm_points2(2,2)], ...
                         'ZData', [current_pos2(3) + arm_points2(3,1), current_pos2(3) + arm_points2(3,2)]);
        set(uav2_body_y, 'XData', [current_pos2(1) + arm_points2(1,3), current_pos2(1) + arm_points2(1,4)], ...
                         'YData', [current_pos2(2) + arm_points2(2,3), current_pos2(2) + arm_points2(2,4)], ...
                         'ZData', [current_pos2(3) + arm_points2(3,3), current_pos2(3) + arm_points2(3,4)]);
        
        % Update UAV1 propellers
        prop_radius = 0.12;
        theta_prop = linspace(0, 2*pi, 16);
        prop_x_circle = prop_radius * cos(theta_prop);
        prop_y_circle = prop_radius * sin(theta_prop);
        prop_z_circle = zeros(size(prop_x_circle));
        
        prop_positions_local = [arm_length, -arm_length, 0, 0; 
                               0, 0, arm_length, -arm_length; 
                               0, 0, 0, 0];
        prop_positions_global1 = R1 * prop_positions_local;
        
        for j = 1:4
            prop_center1 = current_pos1 + prop_positions_global1(:,j)';
            prop_circle_local = [prop_x_circle; prop_y_circle; prop_z_circle];
            prop_circle_global1 = R1 * prop_circle_local;
            
            set(propellers1(j), 'XData', prop_center1(1) + prop_circle_global1(1,:), ...
                               'YData', prop_center1(2) + prop_circle_global1(2,:), ...
                               'ZData', prop_center1(3) + prop_circle_global1(3,:));
        end
        
        % Update UAV2 propellers
        prop_positions_global2 = R2 * prop_positions_local;
        
        for j = 1:4
            prop_center2 = current_pos2 + prop_positions_global2(:,j)';
            prop_circle_global2 = R2 * prop_circle_local;
            
            set(propellers2(j), 'XData', prop_center2(1) + prop_circle_global2(1,:), ...
                               'YData', prop_center2(2) + prop_circle_global2(2,:), ...
                               'ZData', prop_center2(3) + prop_circle_global2(3,:));
        end
        
        % Update payload box position
        current_payload_pos = [Np_traj(i), Ep_traj(i), Dp_traj(i)];
        for j = 1:6
            face_vertices = vertices(faces(j,:),:) + current_payload_pos;
            set(payload_box_faces(j), 'XData', face_vertices(:,1), 'YData', face_vertices(:,2), 'ZData', face_vertices(:,3));
        end
        
        % Update tether lines
        set(tether1_line, 'XData', [N1_traj(i) Np_traj(i)], 'YData', [E1_traj(i) Ep_traj(i)], 'ZData', [D1_traj(i) Dp_traj(i)]);
        set(tether2_line, 'XData', [N2_traj(i) Np_traj(i)], 'YData', [E2_traj(i) Ep_traj(i)], 'ZData', [D2_traj(i) Dp_traj(i)]);
        
        % Update orientation lines for UAV1
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
        x2_ned = R2 * (uav_size*[1;0;0]);
        y2_ned = R2 * (uav_size*[0;1;0]);
        z2_ned = R2 * (uav_size*[0;0;1]);
        set(roll2_line, 'XData', N2_traj(i), 'YData', E2_traj(i), 'ZData', D2_traj(i), ...
                      'UData', x2_ned(1), 'VData', x2_ned(2), 'WData', x2_ned(3));
        set(pitch2_line, 'XData', N2_traj(i), 'YData', E2_traj(i), 'ZData', D2_traj(i), ...
                       'UData', y2_ned(1), 'VData', y2_ned(2), 'WData', y2_ned(3));
        set(yaw2_line, 'XData', N2_traj(i), 'YData', E2_traj(i), 'ZData', D2_traj(i), ...
                     'UData', z2_ned(1), 'VData', z2_ned(2), 'WData', z2_ned(3));

        % Calculate metrics for display
        tether_length1 = norm([N1_traj(i) - Np_traj(i); E1_traj(i) - Ep_traj(i); D1_traj(i) - Dp_traj(i)]);
        tether_length2 = norm([N2_traj(i) - Np_traj(i); E2_traj(i) - Ep_traj(i); D2_traj(i) - Dp_traj(i)]);
        
        % Enhanced title with metrics
        if has_target
            current_center_pos = [center_N_traj(i), center_E_traj(i), center_D_traj(i)];
            distance_to_target = norm(current_center_pos - [Ndes, Edes, Ddes]);
            time_text = sprintf('Time: %.2f s | Target Dist: %.2f m | Tethers: %.2f, %.2f m', ...
                t(i), distance_to_target, tether_length1, tether_length2);
        else
            time_text = sprintf('Time: %.2f s | Tethers: %.2f, %.2f m', ...
                t(i), tether_length1, tether_length2);
        end
        title(['UAV-Payload Animation Replay (NED) - ', time_text]);
        
        % Real-time speed (same as original)
        drawnow;
        pause(0.02);  % Realistic animation speed
    end
    
    if ishandle(f_anim)
        fprintf('Animation replay complete!\n');
    else
        fprintf('Animation stopped by user.\n');
    end
end
