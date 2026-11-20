%% =========================================================================
% Self-Balancing Robot - Pure MATLAB PID Simulator (No Simulink Needed!)
% Change Kp, Ki, Kd below and press "Run" to see the robot stabilize.
% =========================================================================
clear; close all; clc;

%% 1. TUNE YOUR PID GAINS & TEST CONDITIONS HERE
% -------------------------------------------------------------------------
Kp = 22.0;               % Proportional Gain (restoring stiffness)
Ki = 0.4;                % Integral Gain     (steady-state error correction)
Kd = 2.5;                % Derivative Gain   (damping against wobble)

initial_tilt_deg = 8.0;  % Starting offset angle (degrees)
sim_time = 3.0;          % Simulation duration (seconds)

% Robot Physical Properties
L = 0.20;                % Height to Center of Mass (m)
g = 9.81;                % Gravity (m/s^2)
% -------------------------------------------------------------------------

%% 2. Run Non-Linear Physics Simulation (ode45)
theta_0 = deg2rad(initial_tilt_deg);  % Convert to radians
x0 = [theta_0; 0.0; 0.0];             % State: [theta; angular_velocity; integral_error]

% Non-linear inverted pendulum ODE function
ode_fun = @(t, x) [
    x(2);                                                                  % d(theta)/dt = omega
    (g * sin(x(1)) - (Kp * x(1) + Ki * x(3) + Kd * x(2)) * cos(x(1))) / L; % d(omega)/dt = theta_ddot
    x(1)                                                                   % d(int_err)/dt = theta
];

% Simulate with ode45
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-5);
[t, x] = ode45(ode_fun, [0, sim_time], x0, options);

theta_deg = rad2deg(x(:, 1));
omega_deg = rad2deg(x(:, 2));
int_err   = x(:, 3);
u_control = Kp * x(:, 1) + Ki * int_err + Kd * x(:, 2); % Motor command

%% 3. Stability & Performance Metrics
is_stable = all(abs(theta_deg) < 45) && (abs(theta_deg(end)) < 0.5);

% Settling time calculation (within 5% of target)
settled_idx = find(abs(theta_deg) > 0.05 * initial_tilt_deg, 1, 'last');
if isempty(settled_idx)
    settling_time = 0;
elseif settled_idx == length(t)
    settling_time = NaN;
else
    settling_time = t(settled_idx + 1);
end

%% 4. Plot Graphs
figure('Name', 'PID Balancing Simulation', 'Color', 'w', 'Position', [150, 150, 850, 550]);

% --- Subplot 1: Tilt Angle vs Time ---
subplot(2, 1, 1);
plot(t, theta_deg, 'b-', 'LineWidth', 2.2); hold on;
yline(0, 'r--', 'LineWidth', 1.5, 'Label', 'Target Upright (0°)');
yline(initial_tilt_deg, 'k:', 'LineWidth', 1.0, 'Alpha', 0.6);
grid on;
title(sprintf('Robot Pitch Angle \\theta(t)  [Kp=%.1f, Ki=%.2f, Kd=%.2f]', Kp, Ki, Kd), 'FontSize', 12);
xlabel('Time (seconds)', 'FontSize', 10);
ylabel('Tilt Angle (degrees)', 'FontSize', 10);
xlim([0, sim_time]);

% Add status banner
if is_stable
    status_text = sprintf('STATUS: STABLE \\checkmark\nSettling Time: %.2f s', settling_time);
    color_box = [0.1, 0.6, 0.2];
else
    status_text = 'STATUS: UNSTABLE (Fell Over!) \times\nIncrease Kp or Kd';
    color_box = [0.8, 0.1, 0.1];
end
annotation('textbox', [0.70, 0.76, 0.20, 0.12], 'String', status_text, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'w', 'EdgeColor', color_box, ...
    'LineWidth', 1.5, 'FontWeight', 'bold', 'Color', color_box);

% --- Subplot 2: Control Effort (Wheel Acceleration) ---
subplot(2, 1, 2);
plot(t, u_control, 'm-', 'LineWidth', 2.0); hold on;
yline(0, 'k--', 'LineWidth', 1.0);
grid on;
title('Control Effort u(t) [Wheel Acceleration]', 'FontSize', 12);
xlabel('Time (seconds)', 'FontSize', 10);
ylabel('Acceleration (m/s^2)', 'FontSize', 10);
xlim([0, sim_time]);

fprintf('--------------------------------------------------\n');
fprintf('Simulation Finished!\n');
fprintf('  Kp = %.2f | Ki = %.2f | Kd = %.2f\n', Kp, Ki, Kd);
fprintf('  Initial Tilt: %.1f deg\n', initial_tilt_deg);
fprintf('  Final Tilt:   %.3f deg\n', theta_deg(end));
if is_stable
    fprintf('  System is STABLE (Settled in ~%.2f s)\n', settling_time);
else
    fprintf('  WARNING: Robot fell over or did not settle!\n');
end
fprintf('--------------------------------------------------\n');
