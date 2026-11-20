%% =========================================================================
% Self-Balancing Robot: State-Space Matrix Simulation & Pole Analysis
% Direct matrix computation from your handwritten notes!
% =========================================================================
clear; clc; close all;

%% 1. SYSTEM MATRICES FROM YOUR NOTEBOOK (A and B)
% State vector: x = [position; velocity; pitch_angle; pitch_rate]
A = [0,  1,       0,  0;
     0,  0, -14.172,  0;
     0,  0,       0,  1;
     0,  0,  88.578,  0];

B = [0;
     0.978;
     0;
    -3.612];

%% 2. TUNE YOUR FEEDBACK GAINS HERE: u = -K * x
% K = [Kx, Kv, K_theta, K_theta_dot]
% You can change any of these numbers to see how poles and response change!
Kx          = -5.0;    % Position gain
Kv          = -8.0;    % Velocity gain
K_theta     = -60.0;   % Pitch angle gain
K_thetadot  = -10.0;   % Pitch rate gain

K = [Kx, Kv, K_theta, K_thetadot];

%% 3. COMPUTE POLES DIRECTLY FROM MATRICES
% Open-Loop poles: eigenvalues of A
open_poles = eig(A);

% Closed-Loop poles: eigenvalues of (A - B*K)
A_closed = A - B * K;
closed_poles = eig(A_closed);

% Display computed poles in Command Window
fprintf('====================================================\n');
fprintf('COMPUTED OPEN-LOOP POLES (eig(A)):\n');
disp(open_poles);
fprintf('COMPUTED CLOSED-LOOP POLES (eig(A - B*K)):\n');
disp(closed_poles);

if all(real(closed_poles) < 0)
    fprintf('STATUS: SYSTEM IS STABLE! (All poles in LHP)\n');
else
    fprintf('STATUS: SYSTEM IS UNSTABLE! (Poles in RHP)\n');
end
fprintf('====================================================\n');

%% 4. TIME-DOMAIN SIMULATION (Starting from 5 deg tilt)
theta_init = deg2rad(5.0);           % 5 degrees tilt in radians
x0 = [0; 0; theta_init; 0];          % Initial state: [x; x_dot; theta; theta_dot]

t = 0:0.01:3.0;

% Define linear state-space models
sys_open   = ss(A, zeros(4,1), eye(4), 0);
sys_closed = ss(A_closed, zeros(4,1), eye(4), 0);

% Simulate from initial condition
[y_open, t_open]     = initial(sys_open, x0, 0:0.01:0.6);
[y_closed, t_closed] = initial(sys_closed, x0, t);

%% 5. PLOT S-PLANE & TIME RESPONSES
figure('Color', 'w', 'Position', [100, 80, 1050, 600]);

% --- 1. S-PLANE POLE-ZERO MAP ---
subplot(2, 2, [1, 2]);
plot(real(open_poles), imag(open_poles), 'rx', 'MarkerSize', 12, 'LineWidth', 2.5); hold on;
plot(real(closed_poles), imag(closed_poles), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
xline(0, 'k--', 'LineWidth', 2, 'Label', 'Stability Boundary (Real = 0)');
yline(0, 'k:');
grid on;
title('S-Plane: Computed from eig(A) and eig(A - B*K)', 'FontSize', 12);
xlabel('Real Axis \sigma'); ylabel('Imaginary Axis j\omega');
legend('Open-Loop Poles (Unstable)', 'Closed-Loop Poles (Stable)', 'Location', 'northwest');

% --- 2. MOTION BEFORE (Open-Loop Tilt Angle) ---
subplot(2, 2, 3);
plot(t_open, rad2deg(y_open(:, 3)), 'r', 'LineWidth', 2); hold on;
yline(0, 'k--');
grid on;
title('BEFORE: Open-Loop Response (Fails)', 'FontSize', 11);
xlabel('Time (s)'); ylabel('Pitch Angle (deg)');
xlim([0, 0.6]);

% --- 3. MOTION AFTER (Closed-Loop Tilt & Position) ---
subplot(2, 2, 4);
plot(t_closed, rad2deg(y_closed(:, 3)), 'b', 'LineWidth', 2); hold on;
plot(t_closed, y_closed(:, 1), 'm--', 'LineWidth', 1.8);
yline(0, 'k--');
grid on;
title('AFTER: Closed-Loop Stabilization', 'FontSize', 11);
xlabel('Time (s)');
ylabel('Angle (deg) / Position (m)');
legend('Pitch Angle \theta(t) [deg]', 'Position X(t) [m]', 'Location', 'northeast');
xlim([0, 3.0]);