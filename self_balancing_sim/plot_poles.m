clear; clc; close all;

% Open-loop poles (from state-space A matrix: s^2 - 88.58 = 0)
open_poles = [0, 0, -9.41, 9.41];

% Closed-loop poles (after applying controller)
closed_poles = [-6.25 + 4.68i, -6.25 - 4.68i, -0.80 + 0.45i, -0.80 - 0.45i];

% Plot s-plane (pole-zero map)
figure('Color', 'w');
hold on; grid on;

% Plot Open-loop poles (Red Crosses)
plot(real(open_poles), imag(open_poles), 'rx', 'MarkerSize', 12, 'LineWidth', 2.5);

% Plot Closed-loop poles (Green Circles)
plot(real(closed_poles), imag(closed_poles), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');

% Draw axes & stability boundary at Real = 0
xline(0, 'k--', 'LineWidth', 2, 'Label', 'Stability Boundary (Real = 0)');
yline(0, 'k:');

% Labels and title
xlabel('Real Axis \sigma (Stability / Damping)', 'FontSize', 11);
ylabel('Imaginary Axis j\omega (Oscillation Frequency)', 'FontSize', 11);
title('S-Plane: Open-Loop (Unstable) vs Closed-Loop (Stable)', 'FontSize', 12);
legend('Open-Loop Poles', 'Closed-Loop Poles', 'Location', 'northwest');

xlim([-12, 12]);
ylim([-8, 8]);

% Display pole values in command window
disp('--- Open-Loop Poles ---');
disp(open_poles.');
disp('--- Closed-Loop Poles ---');
disp(closed_poles.');
