%% =========================================================================
% Full Cascaded System (Outer Position Loop + Inner Pitch Loop) in Simulink
% Pure native Simulink blocks - 100% compatible with MATLAB Online!
% =========================================================================
clear; clc;

%% 1. TUNE PARAMETERS & INITIAL CONDITIONS HERE
% -------------------------------------------------------------------------
% Initial conditions & Targets
init_pos = 0.0;          % Initial starting position (meters)
target_pos = 1.0;        % Desired target position (meters)
init_pitch = 0.0;        % Initial tilt angle (radians, e.g. 0.05 ~ 2.8 deg)

% Robot physics (URDF)
L = 0.20;                % Height to Center of Mass (m)
g = 9.81;                % Gravity (m/s^2)

% Inner loop: Pitch PID gains
Kp = 22.0;
Ki = 0.4;
Kd = 2.5;

% Outer loop: Position PD gains
Kp_pos = 0.15;
Kd_pos = 0.30;

% Clamps & limits
max_target_pitch = 0.15; % Maximum allowed lean angle (~8.5 deg)
max_vel = 4.0;           % Maximum wheel speed (+/- 4 m/s)
% -------------------------------------------------------------------------

%% 2. Create Simulink Model
model_name = 'full_balancing_system';

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
new_system(model_name);
open_system(model_name);

set_param(model_name, 'StopTime', '8.0');
set_param(model_name, 'Solver', 'ode45');

%% 3. Add Blocks

% --- Outer Loop: Position Control ---
add_block('simulink/Sources/Constant', [model_name '/Target_X'], ...
    'Value', 'target_pos', 'Position', [40, 80, 80, 110]);

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Pos_Err'], ...
    'Inputs', '+-', 'Position', [140, 85, 160, 115]);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_Kp_Pos'], ...
    'Gain', 'Kp_pos', 'Position', [200, 75, 250, 105]);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_Kd_Pos'], ...
    'Gain', 'Kd_pos', 'Position', [200, 135, 250, 165]);

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Outer_PD'], ...
    'Inputs', '++', 'Position', [290, 85, 310, 115]);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_Invert'], ...
    'Gain', '-1', 'Position', [340, 85, 370, 115]);

add_block('simulink/Discontinuities/Saturation', [model_name '/Sat_Lean_Angle'], ...
    'UpperLimit', 'max_target_pitch', 'LowerLimit', '-max_target_pitch', ...
    'Position', [400, 85, 430, 115]);

% --- Inner Loop: Pitch PID Controller ---
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Pitch_Err'], ...
    'Inputs', '+-', 'Position', [480, 85, 500, 115]);

add_block('simulink/Continuous/PID Controller', [model_name '/PID_Controller'], ...
    'P', 'Kp', 'I', 'Ki', 'D', 'Kd', ...
    'Position', [540, 75, 600, 125]);

add_block('simulink/Discontinuities/Saturation', [model_name '/Sat_Speed'], ...
    'UpperLimit', 'max_vel', 'LowerLimit', '-max_vel', ...
    'Position', [640, 85, 670, 115]);

% --- Motor & Physics Dynamics ---
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Motor_Lag'], ...
    'Numerator', '[1]', 'Denominator', '[0.05 1]', ...
    'Position', [710, 85, 760, 115]);

add_block('simulink/Continuous/Derivative', [model_name '/Deriv_Accel'], ...
    'Position', [800, 85, 830, 115]);

% Physics: d2_theta/dt2 = (g/L)*theta - (1/L)*a
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Accel_Torque'], ...
    'Gain', '1/L', 'Position', [870, 85, 920, 115]);

add_block('simulink/Math Operations/Gain', [model_name '/Gain_Gravity'], ...
    'Gain', 'g/L', 'Position', [870, 175, 920, 205]);

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Physics'], ...
    'Inputs', '-+', 'Position', [960, 85, 980, 115]);

% Integrators for Tilt: accel -> rate -> angle
add_block('simulink/Continuous/Integrator', [model_name '/Integ_Theta_Dot'], ...
    'InitialCondition', '0.0', 'Position', [1020, 85, 1050, 115]);

add_block('simulink/Continuous/Integrator', [model_name '/Integ_Theta'], ...
    'InitialCondition', 'init_pitch', 'Position', [1100, 85, 1130, 115]);

% Integrator for Position: speed -> position x
add_block('simulink/Continuous/Integrator', [model_name '/Integ_Pos_X'], ...
    'InitialCondition', 'init_pos', 'Position', [800, 20, 830, 50]);

% Scopes
add_block('simulink/Sinks/Scope', [model_name '/Scope_Position'], 'Position', [900, 20, 930, 50]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_Pitch'],    'Position', [1200, 85, 1230, 115]);

%% 4. Connect Signal Lines

% Outer Loop Wiring
add_line(model_name, 'Target_X/1', 'Sum_Pos_Err/2');
add_line(model_name, 'Integ_Pos_X/1', 'Sum_Pos_Err/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Pos_Err/1', 'Gain_Kp_Pos/1');
add_line(model_name, 'Motor_Lag/1', 'Gain_Kd_Pos/1', 'autorouting', 'on');

add_line(model_name, 'Gain_Kp_Pos/1', 'Sum_Outer_PD/1');
add_line(model_name, 'Gain_Kd_Pos/1', 'Sum_Outer_PD/2');
add_line(model_name, 'Sum_Outer_PD/1', 'Gain_Invert/1');
add_line(model_name, 'Gain_Invert/1', 'Sat_Lean_Angle/1');

% Inner Loop Wiring
add_line(model_name, 'Sat_Lean_Angle/1', 'Sum_Pitch_Err/2');
add_line(model_name, 'Integ_Theta/1', 'Sum_Pitch_Err/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Pitch_Err/1', 'PID_Controller/1');
add_line(model_name, 'PID_Controller/1', 'Sat_Speed/1');

% Velocity to Position & Acceleration
add_line(model_name, 'Sat_Speed/1', 'Motor_Lag/1');
add_line(model_name, 'Motor_Lag/1', 'Integ_Pos_X/1', 'autorouting', 'on');
add_line(model_name, 'Motor_Lag/1', 'Deriv_Accel/1');
add_line(model_name, 'Integ_Pos_X/1', 'Scope_Position/1');

% Physics Wiring
add_line(model_name, 'Deriv_Accel/1', 'Gain_Accel_Torque/1');
add_line(model_name, 'Gain_Accel_Torque/1', 'Sum_Physics/1');
add_line(model_name, 'Integ_Theta/1', 'Gain_Gravity/1', 'autorouting', 'on');
add_line(model_name, 'Gain_Gravity/1', 'Sum_Physics/2');

add_line(model_name, 'Sum_Physics/1', 'Integ_Theta_Dot/1');
add_line(model_name, 'Integ_Theta_Dot/1', 'Integ_Theta/1');
add_line(model_name, 'Integ_Theta/1', 'Scope_Pitch/1');

save_system(model_name);
fprintf('Full system model "%s.slx" created successfully in Simulink!\n', model_name);
