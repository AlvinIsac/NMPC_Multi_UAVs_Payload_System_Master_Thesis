function analyze_mpc_logs()
% ANALYZE_MPC_LOGS - Display input/output data from MPC controller
% 
% This function displays the logged input states and output controls
% from the MPC controller after a Simulink simulation.
%
% Usage: Run this function after your Simulink simulation completes

    % Check if logged data exists in workspace
    if ~evalin('base', 'exist(''mpc_input_states_log'', ''var'')')
        fprintf('No MPC input states log found. Make sure to run Simulink simulation first.\n');
        return;
    end
    
    if ~evalin('base', 'exist(''mpc_output_controls_log'', ''var'')')
        fprintf('No MPC output controls log found. Make sure to run Simulink simulation first.\n');
        return;
    end
    
    if ~evalin('base', 'exist(''mpc_solve_times_log'', ''var'')')
        fprintf('No MPC solve times log found. Make sure to run Simulink simulation first.\n');
        return;
    end
    
    % Get the logged data from base workspace
    input_states = evalin('base', 'mpc_input_states_log');
    output_controls = evalin('base', 'mpc_output_controls_log');
    solve_times = evalin('base', 'mpc_solve_times_log');
    call_count = evalin('base', 'mpc_controller_call_count');
    
    fprintf('=== MPC Controller Log Analysis ===\n');
    fprintf('Total MPC calls: %d\n\n', call_count);
    
    % Display data for each call
    for i = 1:size(input_states, 2)
        fprintf('--- MPC Call %d (Solve time: %.4f s) ---\n', i, solve_times(i));
        
        % Extract input states
        x_in = input_states(:, i);
        
        % UAV1 states (1-12)
        fprintf('UAV1 Position: [N=%.3f, E=%.3f, D=%.3f]\n', x_in(1), x_in(3), x_in(5));
        fprintf('UAV1 Velocity: [vN=%.3f, vE=%.3f, vD=%.3f]\n', x_in(2), x_in(4), x_in(6));
        fprintf('UAV1 Angles: [phi=%.3f, theta=%.3f, psi=%.3f] deg\n', ...
                rad2deg(x_in(7)), rad2deg(x_in(9)), rad2deg(x_in(11)));
        
        % UAV2 states (13-24)
        fprintf('UAV2 Position: [N=%.3f, E=%.3f, D=%.3f]\n', x_in(13), x_in(15), x_in(17));
        fprintf('UAV2 Velocity: [vN=%.3f, vE=%.3f, vD=%.3f]\n', x_in(14), x_in(16), x_in(18));
        fprintf('UAV2 Angles: [phi=%.3f, theta=%.3f, psi=%.3f] deg\n', ...
                rad2deg(x_in(19)), rad2deg(x_in(21)), rad2deg(x_in(23)));
        
        % Payload states (25-30)
        fprintf('Payload Position: [N=%.3f, E=%.3f, D=%.3f]\n', x_in(25), x_in(27), x_in(29));
        fprintf('Payload Velocity: [vN=%.3f, vE=%.3f, vD=%.3f]\n', x_in(26), x_in(28), x_in(30));
        
        % Extract output controls
        u_out = output_controls(:, i);
        
        % UAV1 controls
        fprintf('UAV1 Controls: [T=%.3f, tau_phi=%.3f, tau_theta=%.3f, tau_psi=%.3f]\n', ...
                u_out(1), u_out(2), u_out(3), u_out(4));
        
        % UAV2 controls
        fprintf('UAV2 Controls: [T=%.3f, tau_phi=%.3f, tau_theta=%.3f, tau_psi=%.3f]\n', ...
                u_out(5), u_out(6), u_out(7), u_out(8));
        
        fprintf('\n');
    end
    
    %% Timing Analysis (at the end)
    fprintf('=== MPC TIMING ANALYSIS ===\n');
    avg_solve_time = mean(solve_times);
    min_solve_time = min(solve_times);
    max_solve_time = max(solve_times);
    
    fprintf('Average MPC solve time: %.4f seconds\n', avg_solve_time);
    fprintf('Minimum solve time: %.4f seconds\n', min_solve_time);
    fprintf('Maximum solve time: %.4f seconds\n', max_solve_time);
    fprintf('\n');
    
    % Save data to MAT file for further analysis
    % filename = sprintf('mpc_log_data_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
    % save(filename, 'input_states', 'output_controls', 'solve_times', 'call_count');
    % fprintf('Data saved to: %s\n', filename);
    
    % Optional: Create plots
    % create_mpc_plots(input_states, output_controls, solve_times);
end

function create_mpc_plots(input_states, output_controls, solve_times)
    % Create visualization plots
    
    figure('Name', 'MPC Controller Analysis', 'Position', [100, 100, 1200, 800]);
    
    % Plot 1: UAV positions
    subplot(2,4,1);
    plot(input_states(1,:), input_states(3,:), 'b-o', 'DisplayName', 'UAV1');
    hold on;
    plot(input_states(13,:), input_states(15,:), 'r-o', 'DisplayName', 'UAV2');
    plot(input_states(25,:), input_states(27,:), 'g-o', 'DisplayName', 'Payload');
    xlabel('North (m)'); ylabel('East (m)');
    title('Trajectory (N-E plane)');
    legend; grid on;
    
    % Plot 2: Altitude
    subplot(2,4,2);
    plot(1:size(input_states,2), input_states(5,:), 'b-o', 'DisplayName', 'UAV1');
    hold on;
    plot(1:size(input_states,2), input_states(17,:), 'r-o', 'DisplayName', 'UAV2');
    plot(1:size(input_states,2), input_states(29,:), 'g-o', 'DisplayName', 'Payload');
    xlabel('MPC Call'); ylabel('Down (m)');
    title('Altitude vs Time');
    legend; grid on;
    
    % Plot 3: Thrust commands
    subplot(2,4,3);
    plot(1:size(output_controls,2), output_controls(1,:), 'b-o', 'DisplayName', 'UAV1 Thrust');
    hold on;
    plot(1:size(output_controls,2), output_controls(5,:), 'r-o', 'DisplayName', 'UAV2 Thrust');
    xlabel('MPC Call'); ylabel('Thrust (N)');
    title('Thrust Commands');
    legend; grid on;
    
    % Plot 4: Roll torque commands
    subplot(2,4,4);
    plot(1:size(output_controls,2), output_controls(2,:), 'b-o', 'DisplayName', 'UAV1 Roll');
    hold on;
    plot(1:size(output_controls,2), output_controls(6,:), 'r-o', 'DisplayName', 'UAV2 Roll');
    xlabel('MPC Call'); ylabel('Roll Torque (Nm)');
    title('Roll Torque Commands');
    legend; grid on;
    
    % Plot 5: Pitch torque commands
    subplot(2,4,5);
    plot(1:size(output_controls,2), output_controls(3,:), 'b-o', 'DisplayName', 'UAV1 Pitch');
    hold on;
    plot(1:size(output_controls,2), output_controls(7,:), 'r-o', 'DisplayName', 'UAV2 Pitch');
    xlabel('MPC Call'); ylabel('Pitch Torque (Nm)');
    title('Pitch Torque Commands');
    legend; grid on;
    
    % Plot 6: Yaw torque commands
    subplot(2,4,6);
    plot(1:size(output_controls,2), output_controls(4,:), 'b-o', 'DisplayName', 'UAV1 Yaw');
    hold on;
    plot(1:size(output_controls,2), output_controls(8,:), 'r-o', 'DisplayName', 'UAV2 Yaw');
    xlabel('MPC Call'); ylabel('Yaw Torque (Nm)');
    title('Yaw Torque Commands');
    legend; grid on;
    
    % Plot 7: MPC Solve Times
    subplot(2,4,7);
    plot(1:length(solve_times), solve_times*1000, 'k-o', 'LineWidth', 1.5);
    xlabel('MPC Call'); ylabel('Solve Time (ms)');
    title('MPC Solution Time');
    grid on;
    avg_line = mean(solve_times)*1000;
    hold on;
    plot([1, length(solve_times)], [avg_line, avg_line], 'r--', 'LineWidth', 2, 'DisplayName', sprintf('Avg: %.2f ms', avg_line));
    legend;
    
    % Plot 8: Solve Time Histogram
    subplot(2,4,8);
    histogram(solve_times*1000, 'EdgeColor', 'black');
    xlabel('Solve Time (ms)'); ylabel('Frequency');
    title('Solve Time Distribution');
    grid on;
end
