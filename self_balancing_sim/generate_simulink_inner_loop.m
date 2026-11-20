%% =========================================================================
% Self-Balancing Robot - INNER LOOP ONLY (Pitch Balancing PID)
% Pure native Simulink blocks (100% compatible with MATLAB Online)
% No extra toolboxes or Stateflow required!
% =========================================================================
clear; clc;

%% 1. Physical Parameters (from URDF)
m_body = 2.0;         % Chassis mass (kg)
L = 0.20;             % Distance to Center of Mass (m)
g = 9.81;             % Gravity (m/s^2)
chassis_depth = 0.10;
chassis_height = 0.40;

% Moment of inertia about pivot axle (J = I_cm + m*L^2)
I_cm = (1.0 / 12.0) * m_body * (chassis_depth^2 + chassis_height^2);
J_pivot = I_cm + m_body * (L^2);  % ~0.1083 kg*m^2

% Dynamic equation coefficients:
% theta_ddot = (m*g*L*sin(theta) - m*L*x_ddot*cos(theta)) / J_pivot
k_grav = (m_body * g * L) / J_pivot;      % Gravity torque gain (~36.2)
k_accel = (m_body * L) / J_pivot;         % Inertial wheel coupling (~3.69)

%% 2. Inner Loop PID Parameters (from pid_controller_node.py)
kp = 22.0;
ki = 0.4;
kd = 2.5;

max_integral = 0.20;  % Anti-windup clamp (rad*s)
max_velocity = 4.0;   % Velocity limit (m/s)

% Initial Conditions & Target
target_pitch = 0.0;   % Desired pitch (0 = upright)
init_pitch = 0.10;    % Initial disturbance (~5.7 degrees)

fprintf('1. Parameters loaded into MATLAB Online Workspace.\n');

%% 3. Create Simulink Model
model_name = 'inner_loop_pitch_sim';

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
new_system(model_name);
open_system(model_name);

set_param(model_name, 'StopTime', '5.0');
set_param(model_name, 'Solver', 'ode45');

%% 4. Add Blocks (Standard Simulink Library)

% Desired Pitch
add_block('simulink/Sources/Constant', [model_name '/Target_Pitch'], ...
    'Value', 'target_pitch', 'Position', [40, 100, 80, 130]);

% Pitch Error: (pitch - target_pitch)
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Error'], ...
    'Inputs', '+-', 'Position', [130, 105, 150, 135]);

% --- Proportional Branch ---
add_block('simulink/Math Operations/Gain', [model_name '/Kp_Gain'], ...
    'Gain', 'kp', 'Position', [210, 60, 260, 90]);

% --- Integral Branch (with anti-windup clamping) ---
add_block('simulink/Continuous/Integrator', [model_name '/Integrator_Err'], ...
    'LimitOutput', 'on', 'UpperSaturationLimit', 'max_integral', 'LowerSaturationLimit', '-max_integral', ...
    'Position', [210, 110, 240, 140]);
add_block('simulink/Math Operations/Gain', [model_name '/Ki_Gain'], ...
    'Gain', 'ki', 'Position', [270, 110, 320, 140]);

% --- Derivative Branch (Kd * pitch_rate) ---
add_block('simulink/Math Operations/Gain', [model_name '/Kd_Gain'], ...
    'Gain', 'kd', 'Position', [210, 170, 260, 200]);

% Sum PID outputs: u = Kp*err + Ki*integ + Kd*pitch_rate
add_block('simulink/Math Operations/Sum', [model_name '/Sum_PID'], ...
    'Inputs', '+++', 'Position', [360, 105, 380, 145]);

% Velocity Saturation [-4.0, +4.0] m/s
add_block('simulink/Discontinuities/Saturation', [model_name '/Sat_Velocity'], ...
    'UpperLimit', 'max_velocity', 'LowerLimit', '-max_velocity', ...
    'Position', [410, 110, 440, 140]);

% Motor lag (20ms actuator response) & acceleration derivative
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Motor_Lag'], ...
    'Numerator', '[1]', 'Denominator', '[0.02 1]', ...
    'Position', [470, 110, 520, 140]);

add_block('simulink/Continuous/Derivative', [model_name '/Deriv_Accel'], ...
    'Position', [550, 110, 580, 140]);

% --- Physics Model (Pure Simulink Math Blocks) ---
% Gravity term: k_grav * sin(theta)
add_block('simulink/Math Operations/Trigonometric Function', [model_name '/Sin_Theta'], ...
    'Function', 'sin', 'Position', [700, 40, 730, 70]);
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Grav'], ...
    'Gain', 'k_grav', 'Position', [760, 40, 810, 70]);

% Inertial acceleration term: k_accel * x_ddot * cos(theta)
add_block('simulink/Math Operations/Trigonometric Function', [model_name '/Cos_Theta'], ...
    'Function', 'cos', 'Position', [650, 160, 680, 190]);
add_block('simulink/Math Operations/Product', [model_name '/Prod_Accel'], ...
    'Inputs', '2', 'Position', [720, 120, 750, 150]);
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Accel'], ...
    'Gain', 'k_accel', 'Position', [780, 120, 830, 150]);

% theta_ddot = (Gravity term) - (Acceleration term)
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Dynamics'], ...
    'Inputs', '+-', 'Position', [860, 80, 880, 110]);

% Integrators: theta_ddot -> theta_dot -> theta
add_block('simulink/Continuous/Integrator', [model_name '/Integ_Theta_Dot'], ...
    'InitialCondition', '0.0', 'Position', [920, 80, 950, 110]);
add_block('simulink/Continuous/Integrator', [model_name '/Integ_Theta'], ...
    'InitialCondition', 'init_pitch', 'Position', [990, 80, 1020, 110]);

% Scopes
add_block('simulink/Sinks/Scope', [model_name '/Scope_Pitch'], 'Position', [1070, 80, 1100, 110]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_Speed'], 'Position', [480, 45, 510, 75]);

%% 5. Connect Signal Lines
add_line(model_name, 'Target_Pitch/1', 'Sum_Error/2');
add_line(model_name, 'Integ_Theta/1', 'Sum_Error/1', 'autorouting', 'on');

% Error to Kp and Integral
add_line(model_name, 'Sum_Error/1', 'Kp_Gain/1');
add_line(model_name, 'Sum_Error/1', 'Integrator_Err/1');
add_line(model_name, 'Integrator_Err/1', 'Ki_Gain/1');

% Pitch rate to Kd
add_line(model_name, 'Integ_Theta_Dot/1', 'Kd_Gain/1', 'autorouting', 'on');

% Sum PID
add_line(model_name, 'Kp_Gain/1', 'Sum_PID/1');
add_line(model_name, 'Ki_Gain/1', 'Sum_PID/2');
add_line(model_name, 'Kd_Gain/1', 'Sum_PID/3');

% Velocity & Acceleration
add_line(model_name, 'Sum_PID/1', 'Sat_Velocity/1');
add_line(model_name, 'Sat_Velocity/1', 'Scope_Speed/1');
add_line(model_name, 'Sat_Velocity/1', 'Motor_Lag/1');
add_line(model_name, 'Motor_Lag/1', 'Deriv_Accel/1');

% Gravity Branch (Sin)
add_line(model_name, 'Integ_Theta/1', 'Sin_Theta/1', 'autorouting', 'on');
add_line(model_name, 'Sin_Theta/1', 'Gain_Grav/1');
add_line(model_name, 'Gain_Grav/1', 'Sum_Dynamics/1');

% Accel Branch (Cos * x_ddot)
add_line(model_name, 'Integ_Theta/1', 'Cos_Theta/1', 'autorouting', 'on');
add_line(model_name, 'Cos_Theta/1', 'Prod_Accel/2');
add_line(model_name, 'Deriv_Accel/1', 'Prod_Accel/1');
add_line(model_name, 'Prod_Accel/1', 'Gain_Accel/1');
add_line(model_name, 'Gain_Accel/1', 'Sum_Dynamics/2');

% Dynamics Integrators
add_line(model_name, 'Sum_Dynamics/1', 'Integ_Theta_Dot/1');
add_line(model_name, 'Integ_Theta_Dot/1', 'Integ_Theta/1');
add_line(model_name, 'Integ_Theta/1', 'Scope_Pitch/1');

% Save
save_system(model_name);
fprintf('2. Model "inner_loop_pitch_sim.slx" created successfully in MATLAB Online!\n');
