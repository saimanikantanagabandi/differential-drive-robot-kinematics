%% Real-Time Kinematic Simulation & Interactive Motion of Wheeled Robot
% Project: Differential Drive Wheeled Mobile Robot - Live Kinematics & Analysis
% Description: Combines real-time moving simulation (with interactive controls)
%              AND comprehensive kinematic analysis (live telemetry + full-run 
%              diagnostic plots and performance metrics).

function diff_drive_moving_simulation()
    clear; clc; close all;

    %% 1. Robot Kinematic Parameters
    robot.r = 0.05;           % Wheel radius [m] (50 mm)
    robot.L = 0.20;           % Wheelbase / track gauge [m] (200 mm)
    robot.body_r = 0.12;      % Chassis radius [m]

    %% 2. Simulation State & Time Configuration
    sim.dt = 0.04;            % Simulation time step [s] (25 Hz update rate)
    sim.time_scale = 1.0;     % Real-time multiplier
    sim.running = true;       % Play/pause state
    sim.mode = 'Manual';      % Default mode: 'Manual', 'Figure-8 Path', 'Circular Orbit', 'Waypoint Tracking'
    sim.reset_requested = false;
    
    % Manual control inputs
    sim.manual_v = 0.25;      % Initial linear speed [m/s]
    sim.manual_w = 0.00;      % Initial angular speed [rad/s]

    %% 3. Waypoint Navigation Coordinates
    waypoints = [ 0.0,  0.0;
                  1.5,  0.5;
                  2.5,  2.0;
                  1.0,  3.0;
                 -0.5,  2.0;
                  0.0,  0.0 ];

    %% 4. Initialize Main Dashboard Window
    fig = figure('Name', 'Wheeled Robot Kinematics - Real-Time Simulation & Analysis', ...
                 'NumberTitle', 'off', ...
                 'Color', [0.12 0.13 0.15], ...
                 'Position', [60, 50, 1300, 740], ...
                 'KeyPressFcn', @key_press_callback, ...
                 'Interruptible', 'on');

    % --- Left: Simulation Arena Axes ---
    ax_arena = axes('Parent', fig, 'Position', [0.05, 0.21, 0.52, 0.72], ...
                    'Color', [0.08 0.09 0.11], 'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
    hold(ax_arena, 'on'); grid(ax_arena, 'on'); axis(ax_arena, 'equal');
    xlim(ax_arena, [-1.5, 3.5]); ylim(ax_arena, [-1.5, 3.5]);
    xlabel(ax_arena, 'X Position [m]', 'Color', [0.9 0.9 0.9], 'FontWeight', 'bold');
    ylabel(ax_arena, 'Y Position [m]', 'Color', [0.9 0.9 0.9], 'FontWeight', 'bold');
    title(ax_arena, 'Differential Drive Arena (Live Kinematics)', 'Color', 'w', 'FontSize', 12);

    % Pre-draw reference path
    h_ref_path = plot(ax_arena, 0, 0, 'Color', [0.4 0.4 0.4], 'LineStyle', ':', 'LineWidth', 1.5);
    
    % Dynamic trail
    h_trail = plot(ax_arena, 0, 0, 'Color', [0.2 0.7 1.0], 'LineWidth', 2.0);

    % Robot body visual elements
    th_circ = linspace(0, 2*pi, 36);
    bx = robot.body_r * cos(th_circ);
    by = robot.body_r * sin(th_circ);
    h_chassis = patch('Parent', ax_arena, 'XData', bx, 'YData', by, ...
                      'FaceColor', [0.15 0.45 0.85], 'FaceAlpha', 0.8, ...
                      'EdgeColor', [0.6 0.85 1.0], 'LineWidth', 1.8);
    h_caster = plot(ax_arena, 0, 0, 'o', 'MarkerFaceColor', [0.9 0.9 0.9], ...
                    'MarkerEdgeColor', 'none', 'MarkerSize', 5);
    h_heading = plot(ax_arena, [0, 0], [0, 0], 'Color', [1.0 0.25 0.25], 'LineWidth', 2.5);

    % Wheels
    wheel_w = robot.r * 0.9;
    wheel_t = 0.035;
    w_box = [-wheel_w/2, wheel_w/2, wheel_w/2, -wheel_w/2; ...
             -wheel_t/2, -wheel_t/2, wheel_t/2, wheel_t/2];
    h_wheel_r = patch('Parent', ax_arena, 'XData', [], 'YData', [], ...
                      'FaceColor', [0.85 0.85 0.85], 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1);
    h_wheel_l = patch('Parent', ax_arena, 'XData', [], 'YData', [], ...
                      'FaceColor', [0.85 0.85 0.85], 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1);

    % --- Right: Real-Time Kinematic Telemetry Plots ---
    % Plot 1: Linear & Angular Velocities
    ax_vel = axes('Parent', fig, 'Position', [0.63, 0.60, 0.33, 0.33], ...
                  'Color', [0.08 0.09 0.11], 'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
    hold(ax_vel, 'on'); grid(ax_vel, 'on');
    h_v = plot(ax_vel, 0, 0, 'Color', [0.3 0.8 1.0], 'LineWidth', 1.8, 'DisplayName', 'v [m/s]');
    h_w = plot(ax_vel, 0, 0, 'Color', [1.0 0.7 0.2], 'LineWidth', 1.8, 'DisplayName', '\omega [rad/s]');
    title(ax_vel, 'Chassis Velocities (v, \omega)', 'Color', 'w', 'FontSize', 11);
    xlabel(ax_vel, 'Time Window [s]', 'Color', [0.8 0.8 0.8]);
    legend(ax_vel, 'TextColor', 'w', 'Color', [0.12 0.13 0.15], 'Location', 'northwest');

    % Plot 2: Wheel Actuator Speeds
    ax_wheels = axes('Parent', fig, 'Position', [0.63, 0.21, 0.33, 0.33], ...
                     'Color', [0.08 0.09 0.11], 'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
    hold(ax_wheels, 'on'); grid(ax_wheels, 'on');
    h_wr = plot(ax_wheels, 0, 0, 'Color', [1.0 0.3 0.3], 'LineWidth', 1.8, 'DisplayName', '\omega_R (Right)');
    h_wl = plot(ax_wheels, 0, 0, 'Color', [0.3 1.0 0.4], 'LineWidth', 1.8, 'DisplayName', '\omega_L (Left)');
    title(ax_wheels, 'Actuator Speeds (\omega_R, \omega_L)', 'Color', 'w', 'FontSize', 11);
    xlabel(ax_wheels, 'Time Window [s]', 'Color', [0.8 0.8 0.8]);
    legend(ax_wheels, 'TextColor', 'w', 'Color', [0.12 0.13 0.15], 'Location', 'northwest');

    % --- Top Status Banner ---
    h_status = uicontrol('Parent', fig, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.05, 0.945, 0.91, 0.04], ...
        'BackgroundColor', [0.12 0.13 0.15], 'ForegroundColor', [0.3 1.0 0.5], ...
        'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
        'String', 'Status: Running | Mode: Manual Keyboard Drive');

    %% --- Bottom Control Panel (Interactive Buttons & Sliders) ---
    % 1. Play / Pause & Reset
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Play / Pause', ...
        'Units', 'normalized', 'Position', [0.05, 0.11, 0.08, 0.05], ...
        'BackgroundColor', [0.2 0.5 0.8], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
        'Callback', @toggle_play);

    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Reset State', ...
        'Units', 'normalized', 'Position', [0.135, 0.11, 0.08, 0.05], ...
        'BackgroundColor', [0.7 0.25 0.25], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
        'Callback', @reset_sim);

    % NEW: View Full Kinematic Analysis Report Button
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'View Full Analysis 📊', ...
        'Units', 'normalized', 'Position', [0.22, 0.11, 0.12, 0.05], ...
        'BackgroundColor', [0.55 0.25 0.75], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
        'Callback', @(~,~) generate_analysis_plots());

    % 2. Mode Selector
    uicontrol('Parent', fig, 'Style', 'text', 'String', 'Mode:', ...
        'Units', 'normalized', 'Position', [0.345, 0.11, 0.04, 0.04], ...
        'BackgroundColor', [0.12 0.13 0.15], 'ForegroundColor', 'w', ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'right');

    uicontrol('Parent', fig, 'Style', 'popupmenu', ...
        'String', {'Manual Keyboard / Buttons', 'Figure-8 Path', 'Circular Orbit', 'Waypoint Tracking'}, ...
        'Value', 1, ...
        'Units', 'normalized', 'Position', [0.39, 0.11, 0.14, 0.05], ...
        'BackgroundColor', [0.25 0.26 0.30], 'ForegroundColor', 'w', ...
        'Callback', @change_mode);

    % 3. On-Screen Directional D-Pad Buttons
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '▲ Forward (W)', ...
        'Units', 'normalized', 'Position', [0.59, 0.10, 0.09, 0.05], ...
        'BackgroundColor', [0.2 0.6 0.4], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
        'Callback', @(~,~) btn_drive(0.08, 0));

    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '▼ Reverse (S)', ...
        'Units', 'normalized', 'Position', [0.59, 0.03, 0.09, 0.05], ...
        'BackgroundColor', [0.2 0.6 0.4], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
        'Callback', @(~,~) btn_drive(-0.08, 0));

    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '◄ Left (A)', ...
        'Units', 'normalized', 'Position', [0.54, 0.03, 0.048, 0.05], ...
        'BackgroundColor', [0.8 0.5 0.1], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
        'Callback', @(~,~) btn_drive(0, 0.20));

    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Right (D) ►', ...
        'Units', 'normalized', 'Position', [0.682, 0.03, 0.048, 0.05], ...
        'BackgroundColor', [0.8 0.5 0.1], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
        'Callback', @(~,~) btn_drive(0, -0.20));

    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '■ Stop (Space)', ...
        'Units', 'normalized', 'Position', [0.735, 0.03, 0.08, 0.12], ...
        'BackgroundColor', [0.75 0.2 0.2], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
        'Callback', @(~,~) btn_stop());

    % 4. Speed & Steering Sliders
    uicontrol('Parent', fig, 'Style', 'text', 'String', 'Linear Speed v [m/s]:', ...
        'Units', 'normalized', 'Position', [0.05, 0.04, 0.12, 0.03], ...
        'BackgroundColor', [0.12 0.13 0.15], 'ForegroundColor', [0.3 0.8 1.0], ...
        'FontSize', 9, 'HorizontalAlignment', 'left');
    h_sld_v = uicontrol('Parent', fig, 'Style', 'slider', ...
        'Min', -0.5, 'Max', 0.8, 'Value', sim.manual_v, ...
        'Units', 'normalized', 'Position', [0.18, 0.04, 0.10, 0.03], ...
        'Callback', @slider_v_callback);

    uicontrol('Parent', fig, 'Style', 'text', 'String', 'Steering \omega [rad/s]:', ...
        'Units', 'normalized', 'Position', [0.29, 0.04, 0.12, 0.03], ...
        'BackgroundColor', [0.12 0.13 0.15], 'ForegroundColor', [1.0 0.7 0.2], ...
        'FontSize', 9, 'HorizontalAlignment', 'left');
    h_sld_w = uicontrol('Parent', fig, 'Style', 'slider', ...
        'Min', -1.5, 'Max', 1.5, 'Value', sim.manual_w, ...
        'Units', 'normalized', 'Position', [0.42, 0.04, 0.08, 0.03], ...
        'Callback', @slider_w_callback);

    % Tip box
    uicontrol('Parent', fig, 'Style', 'text', ...
        'String', 'Tip: Click "View Full Analysis" anytime to see complete state plots and metrics!', ...
        'Units', 'normalized', 'Position', [0.825, 0.03, 0.16, 0.12], ...
        'BackgroundColor', [0.12 0.13 0.15], 'ForegroundColor', [0.75 0.85 1.0], ...
        'FontSize', 9, 'HorizontalAlignment', 'left');

    %% 5. Full-Run Data Logger (Stores Entire Simulation History for Kinematic Analysis)
    history.t = 0;
    history.x = 0;
    history.y = 0;
    history.theta = 0;
    history.v = sim.manual_v;
    history.w = sim.manual_w;
    history.wr = (sim.manual_v + (robot.L / 2) * sim.manual_w) / robot.r;
    history.wl = (sim.manual_v - (robot.L / 2) * sim.manual_w) / robot.r;

    %% 6. Master Simulation Execution Loop
    q = [0.0; 0.0; 0.0];      % [x; y; theta]
    t = 0.0;
    wp_idx = 1;
    
    buf_len = 150;
    buf_t   = zeros(1, buf_len);
    buf_v   = zeros(1, buf_len);
    buf_w   = zeros(1, buf_len);
    buf_wr  = zeros(1, buf_len);
    buf_wl  = zeros(1, buf_len);
    
    trail_x = q(1);
    trail_y = q(2);

    update_reference_path();

    while ishandle(fig)
        loop_start = tic;

        if sim.reset_requested
            q = [0.0; 0.0; 0.0];
            t = 0.0;
            wp_idx = 1;
            trail_x = 0; trail_y = 0;
            buf_t(:) = 0; buf_v(:) = 0; buf_w(:) = 0;
            buf_wr(:) = 0; buf_wl(:) = 0;
            sim.manual_v = 0.25; sim.manual_w = 0.0;
            set(h_sld_v, 'Value', sim.manual_v);
            set(h_sld_w, 'Value', sim.manual_w);
            
            % Reset history
            history.t = 0; history.x = 0; history.y = 0; history.theta = 0;
            history.v = sim.manual_v; history.w = sim.manual_w;
            history.wr = (sim.manual_v + (robot.L / 2) * sim.manual_w) / robot.r;
            history.wl = (sim.manual_v - (robot.L / 2) * sim.manual_w) / robot.r;
            
            sim.reset_requested = false;
            update_reference_path();
        end

        if sim.running
            % Determine velocities (v, w)
            switch sim.mode
                case 'Manual'
                    v_cmd = sim.manual_v;
                    w_cmd = sim.manual_w;
                case 'Figure-8 Path'
                    v_cmd = 0.35; 
                    w_cmd = 0.45 * sin(0.4 * t);
                case 'Circular Orbit'
                    v_cmd = 0.30;
                    w_cmd = 0.30;
                case 'Waypoint Tracking'
                    target = waypoints(wp_idx, :);
                    dx = target(1) - q(1);
                    dy = target(2) - q(2);
                    rho = sqrt(dx^2 + dy^2);
                    
                    if rho < 0.15
                        wp_idx = mod(wp_idx, size(waypoints, 1)) + 1;
                        target = waypoints(wp_idx, :);
                        dx = target(1) - q(1);
                        dy = target(2) - q(2);
                        rho = sqrt(dx^2 + dy^2);
                    end
                    
                    target_heading = atan2(dy, dx);
                    heading_err = atan2(sin(target_heading - q(3)), cos(target_heading - q(3)));
                    
                    v_cmd = min(0.35, 0.4 * rho) * max(0, cos(heading_err));
                    w_cmd = 1.5 * heading_err;
            end

            % Inverse Kinematics
            omega_R = (v_cmd + (robot.L / 2) * w_cmd) / robot.r;
            omega_L = (v_cmd - (robot.L / 2) * w_cmd) / robot.r;

            % Forward Kinematic Integration (RK4)
            dt = sim.dt;
            k1 = kinematics(q, v_cmd, w_cmd);
            k2 = kinematics(q + 0.5 * dt * k1, v_cmd, w_cmd);
            k3 = kinematics(q + 0.5 * dt * k2, v_cmd, w_cmd);
            k4 = kinematics(q + dt * k3, v_cmd, w_cmd);

            q = q + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
            q(3) = atan2(sin(q(3)), cos(q(3)));
            
            t = t + dt;

            % Log to full history
            history.t(end+1)     = t;
            history.x(end+1)     = q(1);
            history.y(end+1)     = q(2);
            history.theta(end+1) = q(3);
            history.v(end+1)     = v_cmd;
            history.w(end+1)     = w_cmd;
            history.wr(end+1)    = omega_R;
            history.wl(end+1)    = omega_L;

            % Rolling buffers for live telemetry
            trail_x(end+1) = q(1);
            trail_y(end+1) = q(2);
            if length(trail_x) > 400
                trail_x(1) = [];
                trail_y(1) = [];
            end

            buf_t  = [buf_t(2:end), t];
            buf_v  = [buf_v(2:end), v_cmd];
            buf_w  = [buf_w(2:end), w_cmd];
            buf_wr = [buf_wr(2:end), omega_R];
            buf_wl = [buf_wl(2:end), omega_L];

            % Update Graphics
            xk = q(1); yk = q(2); th = q(3);
            R = [cos(th), -sin(th); sin(th), cos(th)];

            % Chassis
            chassis_rot = R * [bx; by];
            set(h_chassis, 'XData', chassis_rot(1, :) + xk, 'YData', chassis_rot(2, :) + yk);

            % Heading line & caster
            head_vec = R * [robot.body_r * 1.35; 0];
            set(h_heading, 'XData', [xk, xk + head_vec(1)], 'YData', [yk, yk + head_vec(2)]);
            
            caster_pt = R * [robot.body_r * 0.75; 0];
            set(h_caster, 'XData', xk + caster_pt(1), 'YData', yk + caster_pt(2));

            % Wheels
            wr_local = w_box + [0; -robot.L/2];
            wl_local = w_box + [0;  robot.L/2];
            wr_world = R * wr_local + [xk; yk];
            wl_world = R * wl_local + [xk; yk];
            set(h_wheel_r, 'XData', wr_world(1, :), 'YData', wr_world(2, :));
            set(h_wheel_l, 'XData', wl_world(1, :), 'YData', wl_world(2, :));

            % Trail
            set(h_trail, 'XData', trail_x, 'YData', trail_y);

            % Dynamic Arena Centering
            margin = 1.6;
            xlim(ax_arena, [xk - margin, xk + margin]);
            ylim(ax_arena, [yk - margin, yk + margin]);

            % Telemetry Charts
            set(h_v, 'XData', buf_t, 'YData', buf_v);
            set(h_w, 'XData', buf_t, 'YData', buf_w);
            xlim(ax_vel, [min(buf_t), max(buf_t) + 0.1]);
            ylim(ax_vel, [-0.6, 1.0]);

            set(h_wr, 'XData', buf_t, 'YData', buf_wr);
            set(h_wl, 'XData', buf_t, 'YData', buf_wl);
            xlim(ax_wheels, [min(buf_t), max(buf_t) + 0.1]);
            ylim(ax_wheels, [min([min(buf_wr), min(buf_wl), -2]) - 0.5, ...
                            max([max(buf_wr), max(buf_wl), 8]) + 0.5]);

            % Update Status Text
            set(h_status, 'String', sprintf(...
                'Time: %.2fs | Pose: [X: %.2fm, Y: %.2fm, \\theta: %.1f°] | v: %.2fm/s, \\omega: %.2frad/s | \\omega_R: %.1frad/s, \\omega_L: %.1frad/s', ...
                t, xk, yk, rad2deg(th), v_cmd, w_cmd, omega_R, omega_L));
        end

        drawnow limitrate;

        elapsed = toc(loop_start);
        target_sleep = (sim.dt / sim.time_scale) - elapsed;
        if target_sleep > 0
            pause(target_sleep);
        end
    end

    %% Full Kinematic Analysis Reporting Function
    function generate_analysis_plots()
        if length(history.t) < 5
            fprintf('Need at least a few seconds of motion history before generating analysis.\n');
            return;
        end

        % Create a separate high-resolution analytical figure
        h_fig_an = figure('Name', 'Complete Kinematic Analysis Report', ...
                          'Color', 'w', 'Position', [150, 100, 1050, 680]);

        % 1. Full 2D Trajectory with start/end markers
        subplot(2, 2, 1);
        plot(history.x, history.y, 'b-', 'LineWidth', 2); hold on;
        plot(history.x(1), history.y(1), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
        plot(history.x(end), history.y(end), 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
        grid on; axis equal;
        xlabel('X Position [m]', 'FontWeight', 'bold');
        ylabel('Y Position [m]', 'FontWeight', 'bold');
        title('Full Trajectory (X-Y)', 'FontSize', 11);
        legend('Actual Path', 'Start', 'Current End', 'Location', 'best');

        % 2. Full State Variables vs Time (x, y, theta)
        subplot(2, 2, 2);
        plot(history.t, history.x, 'r-', 'LineWidth', 1.6); hold on;
        plot(history.t, history.y, 'g-', 'LineWidth', 1.6);
        plot(history.t, history.theta, 'b--', 'LineWidth', 1.6);
        grid on;
        xlabel('Time [s]', 'FontWeight', 'bold');
        ylabel('State Value', 'FontWeight', 'bold');
        title('States: x(t), y(t), \theta(t)', 'FontSize', 11);
        legend('x [m]', 'y [m]', '\theta [rad]', 'Location', 'best');

        % 3. Full Chassis Control Velocities (v, omega)
        subplot(2, 2, 3);
        yyaxis left;
        plot(history.t, history.v, 'b-', 'LineWidth', 1.6);
        ylabel('Linear Velocity v [m/s]', 'FontWeight', 'bold');
        yyaxis right;
        plot(history.t, history.w, 'm--', 'LineWidth', 1.6);
        ylabel('Angular Velocity \omega [rad/s]', 'FontWeight', 'bold');
        grid on;
        xlabel('Time [s]', 'FontWeight', 'bold');
        title('Chassis Velocities vs Time', 'FontSize', 11);

        % 4. Full Actuator Wheel Velocities (omega_R, omega_L)
        subplot(2, 2, 4);
        plot(history.t, history.wr, 'r-', 'LineWidth', 1.6); hold on;
        plot(history.t, history.wl, 'k--', 'LineWidth', 1.6);
        grid on;
        xlabel('Time [s]', 'FontWeight', 'bold');
        ylabel('Angular Velocity [rad/s]', 'FontWeight', 'bold');
        title('Wheel Speeds: \omega_R(t), \omega_L(t)', 'FontSize', 11);
        legend('\omega_R (Right)', '\omega_L (Left)', 'Location', 'best');

        % Calculate and print numerical project metrics
        total_time = history.t(end) - history.t(1);
        dx = diff(history.x);
        dy = diff(history.y);
        path_length = sum(sqrt(dx.^2 + dy.^2));
        net_disp = sqrt((history.x(end)-history.x(1))^2 + (history.y(end)-history.y(1))^2);
        
        fprintf('\n=========================================\n');
        fprintf('   WHEELED ROBOT KINEMATIC ANALYSIS      \n');
        fprintf('=========================================\n');
        fprintf('Total Simulation Time : %.2f s\n', total_time);
        fprintf('Total Traveled Distance: %.2f m\n', path_length);
        fprintf('Net Displacement       : %.2f m\n', net_disp);
        fprintf('Max Linear Velocity    : %.2f m/s\n', max(history.v));
        fprintf('Max Angular Velocity   : %.2f rad/s\n', max(abs(history.w)));
        fprintf('Max Right Wheel Speed  : %.2f rad/s (%.1f RPM)\n', max(abs(history.wr)), max(abs(history.wr))*60/(2*pi));
        fprintf('Max Left Wheel Speed   : %.2f rad/s (%.1f RPM)\n', max(abs(history.wl)), max(abs(history.wl))*60/(2*pi));
        fprintf('=========================================\n\n');
    end

    %% Callback & Helper Functions
    function dq = kinematics(q_in, v_in, w_in)
        theta_val = q_in(3);
        dq = [v_in * cos(theta_val); ...
              v_in * sin(theta_val); ...
              w_in];
    end

    function update_reference_path()
        switch sim.mode
            case 'Figure-8 Path'
                t_eval = linspace(0, 4*pi/0.4, 250);
                x_ref = zeros(size(t_eval));
                y_ref = zeros(size(t_eval));
                th_ref = 0; x_curr = 0; y_curr = 0;
                for i = 1:length(t_eval)
                    v_i = 0.35; w_i = 0.45 * sin(0.4 * t_eval(i));
                    th_ref = th_ref + w_i * (t_eval(2)-t_eval(1));
                    x_curr = x_curr + v_i * cos(th_ref) * (t_eval(2)-t_eval(1));
                    y_curr = y_curr + v_i * sin(th_ref) * (t_eval(2)-t_eval(1));
                    x_ref(i) = x_curr; y_ref(i) = y_curr;
                end
                set(h_ref_path, 'XData', x_ref, 'YData', y_ref, 'LineStyle', ':', 'Marker', 'none', 'Visible', 'on');
            case 'Circular Orbit'
                th_arr = linspace(0, 2*pi, 100);
                set(h_ref_path, 'XData', 1.0 * sin(th_arr), 'YData', 1.0 * (1 - cos(th_arr)), 'LineStyle', ':', 'Marker', 'none', 'Visible', 'on');
            case 'Waypoint Tracking'
                set(h_ref_path, 'XData', waypoints(:, 1), 'YData', waypoints(:, 2), ...
                    'LineStyle', '--', 'Marker', 's', 'MarkerFaceColor', [1 0.6 0.2], 'Visible', 'on');
            case 'Manual'
                set(h_ref_path, 'Visible', 'off');
        end
    end

    function toggle_play(~, ~)
        sim.running = ~sim.running;
        focus_fig();
    end

    function reset_sim(~, ~)
        sim.reset_requested = true;
        focus_fig();
    end

    function change_mode(src, ~)
        modes = src.String;
        sim.mode = modes{src.Value};
        sim.reset_requested = true;
        focus_fig();
    end

    function btn_drive(dv, dw)
        sim.manual_v = max(-0.5, min(0.8, sim.manual_v + dv));
        sim.manual_w = max(-1.5, min(1.5, sim.manual_w + dw));
        set(h_sld_v, 'Value', sim.manual_v);
        set(h_sld_w, 'Value', sim.manual_w);
        focus_fig();
    end

    function btn_stop()
        sim.manual_v = 0;
        sim.manual_w = 0;
        set(h_sld_v, 'Value', 0);
        set(h_sld_w, 'Value', 0);
        focus_fig();
    end

    function slider_v_callback(src, ~)
        sim.manual_v = src.Value;
        focus_fig();
    end

    function slider_w_callback(src, ~)
        sim.manual_w = src.Value;
        focus_fig();
    end

    function key_press_callback(~, event)
        if ~strcmp(sim.mode, 'Manual'), return; end
        switch lower(event.Key)
            case {'w', 'uparrow'}
                sim.manual_v = min(0.80, sim.manual_v + 0.05);
            case {'s', 'downarrow'}
                sim.manual_v = max(-0.40, sim.manual_v - 0.05);
            case {'a', 'leftarrow'}
                sim.manual_w = min(1.50, sim.manual_w + 0.15);
            case {'d', 'rightarrow'}
                sim.manual_w = max(-1.50, sim.manual_w - 0.15);
            case 'space'
                sim.manual_v = 0;
                sim.manual_w = 0;
        end
        set(h_sld_v, 'Value', sim.manual_v);
        set(h_sld_w, 'Value', sim.manual_w);
    end

    function focus_fig()
        figure(fig);
    end
end
