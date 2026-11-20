clear; clc; close all;

% Robot parameters
L = 0.2;          % distance to center of mass (m)
g = 9.81;         % gravity (m/s^2)

% PID gains (change these to test stability)
Kp = 22.0;
Ki = 0.4;
Kd = 2.5;

% Simulation settings
dt = 0.01;        % time step (100 Hz, same as ROS node)
t = 0:dt:3;       % 3 seconds simulation
N = length(t);

% Arrays to store data
theta = zeros(1, N);
theta_dot = zeros(1, N);
u = zeros(1, N);

% Initial tilt angle (starts tilted at 10 degrees)
theta(1) = deg2rad(10);
integral_err = 0;

% Main simulation loop
for i = 1:N-1
    % PID control law
    err = theta(i);
    integral_err = integral_err + err * dt;
    u(i) = Kp * err + Ki * integral_err + Kd * theta_dot(i);
    
    % Inverted pendulum dynamics (acceleration)
    theta_ddot = (g * sin(theta(i)) - u(i) * cos(theta(i))) / L;
    
    % Update velocity and angle
    theta_dot(i+1) = theta_dot(i) + theta_ddot * dt;
    theta(i+1) = theta(i) + theta_dot(i+1) * dt;
end

% Plot results
figure;
plot(t, rad2deg(theta), 'b', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Pitch Angle (degrees)');
title('Self-Balancing Robot - Pitch Stabilization');
yline(0, '--r', 'Upright');