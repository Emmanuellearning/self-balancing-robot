clear; clc; close all;

%% 1. Pole Locations
open_poles   = [0, 0, -9.41, 9.41];                                      % Open loop
closed_poles = [-6.25 + 4.68i, -6.25 - 4.68i, -0.80 + 0.45i, -0.80 - 0.45i]; % Closed loop

%% 2. Time Responses (Starting from 5 degree tilt)
t = 0:0.01:2.0;
theta0 = 5.0; % degrees

% Open-loop: Exponential fall (theta(t) = theta0 * cosh(9.41*t))
theta_open = theta0 * cosh(9.41 * t);

% Closed-loop: Damped stabilization
theta_closed = theta0 * exp(-6.25 * t) .* (cos(4.68 * t) + (6.25/4.68)*sin(4.68 * t));

%% 3. Plot Comparison Figure
figure('Color', 'w', 'Position', [100, 80, 1000, 600]);

% --- TOP-LEFT: S-Plane BEFORE ---
subplot(2, 2, 1);
plot(real(open_poles), imag(open_poles), 'rx', 'MarkerSize', 12, 'LineWidth', 2.5);
hold on; grid on;
xline(0, 'k--', 'LineWidth', 1.5);
yline(0, 'k:');
title('1. S-Plane BEFORE (Open Loop)', 'FontSize', 11);
xlabel('Real Axis \sigma'); ylabel('Imaginary Axis j\omega');
xlim([-12, 12]); ylim([-8, 8]);
text(9.41, 1.5, '\leftarrow UNSTABLE (+9.41)', 'Color', 'r', 'FontWeight', 'bold');

% --- TOP-RIGHT: S-Plane AFTER ---
subplot(2, 2, 2);
plot(real(closed_poles), imag(closed_poles), 'go', 'MarkerSize', 9, 'LineWidth', 2, 'MarkerFaceColor', 'g');
hold on; grid on;
xline(0, 'k--', 'LineWidth', 1.5);
yline(0, 'k:');
title('2. S-Plane AFTER (Closed Loop)', 'FontSize', 11);
xlabel('Real Axis \sigma'); ylabel('Imaginary Axis j\omega');
xlim([-12, 12]); ylim([-8, 8]);
text(-6.25, 5.5, '\leftarrow Stable Balance (-6.25 \pm 4.7j)', 'Color', [0 0.5 0], 'FontWeight', 'bold');

% --- BOTTOM-LEFT: Time Response BEFORE ---
subplot(2, 2, 3);
plot(t(theta_open <= 90), theta_open(theta_open <= 90), 'r', 'LineWidth', 2);
hold on; grid on;
yline(0, 'k--');
title('3. Motion BEFORE: Robot Falls Over', 'FontSize', 11);
xlabel('Time (s)'); ylabel('Pitch Angle (degrees)');
xlim([0, 1.0]); ylim([0, 90]);
text(0.35, 60, 'Fell to ground (90°)', 'Color', 'r', 'FontWeight', 'bold');

% --- BOTTOM-RIGHT: Time Response AFTER ---
subplot(2, 2, 4);
plot(t, theta_closed, 'b', 'LineWidth', 2);
hold on; grid on;
yline(0, 'r--', 'Upright (0°)');
grid on;
title('4. Motion AFTER: Robot Balances', 'FontSize', 11);
xlabel('Time (s)'); ylabel('Pitch Angle (degrees)');
xlim([0, 1.5]); ylim([-2, 6]);
text(0.6, 0.8, 'Balanced at 0° in 0.7s \checkmark', 'Color', [0 0.5 0], 'FontWeight', 'bold');
