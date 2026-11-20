clear; clc; close all;

% Robot physical parameters
L = 0.2;           % distance to center of mass (m)
g = 9.81;          % gravity (m/s^2)

% Inner Loop: Pitch PID gains
Kp = 22.0;
Ki = 0.4;
Kd = 2.5;

% Outer Loop: Position PD gains
Kp_pos = 0.15;
Kd_pos = 0.30;

% Target position
target_x = 1.0;    % want the robot to drive to 1.0 meter and stop

% Simulation settings
dt = 0.01;         % 100 Hz (matches ROS node)
t = 0:dt:6;        % 6 seconds
N = length(t);

% State arrays
theta = zeros(1, N);       % pitch angle (rad)
theta_dot = zeros(1, N);   % pitch rate (rad/s)
pos_x = zeros(1, N);       % position x (m)
vel_x = zeros(1, N);       % velocity x (m/s)

integral_err = 0;

for i = 1:N-1
    % 1. Outer Loop: calculate desired lean angle to reach target_x
    pos_error = pos_x(i) - target_x;
    target_pitch = -(Kp_pos * pos_error + Kd_pos * vel_x(i));
    
    % Clamp desired lean to +/- 8.5 degrees (0.15 rad)
    target_pitch = max(-0.15, min(0.15, target_pitch));
    
    % 2. Inner Loop: PID control on pitch error
    pitch_error = theta(i) - target_pitch;
    integral_err = integral_err + pitch_error * dt;
    integral_err = max(-0.2, min(0.2, integral_err));  % anti-windup
    
    cmd_v = Kp * pitch_error + Ki * integral_err + Kd * theta_dot(i);
    cmd_v = max(-4.0, min(4.0, cmd_v));                % speed limit (+/- 4 m/s)
    
    % 3. Physics / Dynamics update
    % Wheel motor acceleration
    a = (cmd_v - vel_x(i)) / 0.05;
    
    % Inverted pendulum angular acceleration
    theta_ddot = (g * sin(theta(i)) - a * cos(theta(i))) / L;
    
    % Integrate motion
    theta_dot(i+1) = theta_dot(i) + theta_ddot * dt;
    theta(i+1)     = theta(i) + theta_dot(i+1) * dt;
    vel_x(i+1)     = vel_x(i) + a * dt;
    pos_x(i+1)     = pos_x(i) + vel_x(i+1) * dt;
end

% Plotting results
figure;

subplot(2, 1, 1);
plot(t, pos_x, 'b', 'LineWidth', 2); hold on;
yline(target_x, '--r', 'Target (1.0 m)');
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Robot Position X(t)');

subplot(2, 1, 2);
plot(t, rad2deg(theta), 'm', 'LineWidth', 2); hold on;
yline(0, '--r', 'Upright (0°)');
grid on;
xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Robot Pitch Angle \theta(t)');