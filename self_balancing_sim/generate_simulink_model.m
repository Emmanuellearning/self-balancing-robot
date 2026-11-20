%% =========================================================================
% Self-Balancing Robot - Simulink Model Setup & Generator
% Matches ROS 2 parameters from urdf/self_balancing_robot.urdf.xacro
% and pid_controller_node.py
% =========================================================================
clear; clc;

%% 1. Physical Parameters (from URDF)
m_body = 2.0;         % Chassis mass (kg)
m_wheels = 0.5;       % Total wheels mass (2 x 0.25 kg)
L = 0.20;             % Distance from wheel axle to Center of Mass (m)
g = 9.81;             % Gravity (m/s^2)
r = 0.06;             % Wheel radius (m)

% Body pitch inertia about CoM: Iyy = (1/12)*m*(depth^2 + height^2)
chassis_depth = 0.10;
chassis_height = 0.40;
I_body = (1.0 / 12.0) * m_body * (chassis_depth^2 + chassis_height^2); % ~0.0283 kg*m^2
J_pivot = I_body + m_body * L^2;                                       % Total inertia about pivot

%% 2. Controller Gains (from pid_controller_node.py)
% Inner loop: Pitch PID gains
kp_pitch = 22.0;
ki_pitch = 0.4;
kd_pitch = 2.5;

% Outer loop: Position PD gains
kp_pos = 0.15;
kd_pos = 0.30;

% Clamping & Saturation Limits
max_target_pitch = 0.15;   % +/- 8.5 degrees (rad)
max_integral = 0.20;       % Anti-windup limit (rad*s)
max_vel = 4.0;             % Max wheel velocity command (m/s)

% Initial conditions
init_pitch = 0.05;         % Initial tilt perturbation (rad, ~2.8 degrees)
target_x = 1.0;            % Target position setpoint (m)

fprintf('Parameters loaded into MATLAB Workspace.\n');
fprintf('Opening / Creating Simulink model...\n');

%% 3. Programmatic Simulink Model Generator
model_name = 'self_balancing_robot_sim';

% Close if already open, create fresh model
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
new_system(model_name);
open_system(model_name);

% Configure solver settings
set_param(model_name, 'StopTime', '10.0');
set_param(model_name, 'Solver', 'ode45');

%% 4. Add Blocks
% Target Position
add_block('simulink/Sources/Step', [model_name '/Target_X'], ...
    'Time', '1.0', 'Before', '0', 'After', num2str(target_x), ...
    'Position', [50, 80, 80, 110]);

% Outer Loop Sum (pos_error = pos_x - target_x)
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Pos'], ...
    'Inputs', '+-', 'Position', [140, 85, 160, 115]);

% Position PD Gains
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Kp_Pos'], ...
    'Gain', 'kp_pos', 'Position', [200, 75, 250, 105]);
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Kd_Pos'], ...
    'Gain', 'kd_pos', 'Position', [200, 135, 250, 165]);

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Outer'], ...
    'Inputs', '++', 'Position', [290, 85, 310, 115]);

add_block('simulink/Math Operations/Gain', [model_name '/Invert_Target_Pitch'], ...
    'Gain', '-1', 'Position', [340, 85, 370, 115]);

% Saturate Target Pitch (+/- 0.15 rad)
add_block('simulink/Discontinuities/Saturation', [model_name '/Sat_Target_Pitch'], ...
    'UpperLimit', 'max_target_pitch', 'LowerLimit', '-max_target_pitch', ...
    'Position', [400, 85, 430, 115]);

% Inner Loop Sum (pitch_error = pitch - target_pitch)
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Pitch'], ...
    'Inputs', '+-', 'Position', [480, 85, 500, 115]);

% Pitch PID Branches
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Kp_Pitch'], ...
    'Gain', 'kp_pitch', 'Position', [540, 50, 590, 80]);

% Integral branch with anti-windup clamping
add_block('simulink/Continuous/Integrator', [model_name '/Integrator_Pitch_Err'], ...
    'LimitOutput', 'on', 'UpperSaturationLimit', 'max_integral', 'LowerSaturationLimit', '-max_integral', ...
    'Position', [540, 100, 570, 130]);
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Ki_Pitch'], ...
    'Gain', 'ki_pitch', 'Position', [600, 100, 650, 130]);

% Derivative branch on pitch_rate
add_block('simulink/Math Operations/Gain', [model_name '/Gain_Kd_Pitch'], ...
    'Gain', 'kd_pitch', 'Position', [540, 150, 590, 180]);

% Inner Loop Sum (u = Kp*err + Ki*integ + Kd*pitch_rate)
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Inner'], ...
    'Inputs', '+++', 'Position', [680, 90, 700, 130]);

% Velocity Saturation (+/- 4 m/s)
add_block('simulink/Discontinuities/Saturation', [model_name '/Sat_Velocity'], ...
    'UpperLimit', 'max_vel', 'LowerLimit', '-max_vel', ...
    'Position', [730, 95, 760, 125]);

%% Plant Subsystem (Nonlinear Inverted Pendulum Dynamics)
% Dynamics equation:
% theta_ddot = (m_body * g * L * sin(theta) - m_body * L * x_ddot * cos(theta)) / J_pivot
% x_dot is tracked by diff-drive actuators with bandwidth tau_motor
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Motor_Lag'], ...
    'Numerator', '[1]', 'Denominator', '[0.02 1]', ... % 20ms actuator time constant
    'Position', [800, 95, 860, 125]);

add_block('simulink/Continuous/Derivative', [model_name '/Accel_Deriv'], ...
    'Position', [900, 95, 930, 125]);

% MATLAB Function block for Equations of Motion
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/Pendulum_Dynamics'], ...
    'Position', [980, 85, 1070, 135]);

% Set function code
matlabFunctionBlock = find(slroot, '-isa', 'Stateflow.EMChart', 'Path', [model_name '/Pendulum_Dynamics']);
matlabFunctionBlock.Script = sprintf([...
    'function theta_ddot = fcn(theta, theta_dot, x_ddot, m_body, L, g, J_pivot)\n' ...
    '%% Non-linear inverted pendulum equation\n' ...
    'theta_ddot = (m_body * g * L * sin(theta) - m_body * L * x_ddot * cos(theta)) / J_pivot;\n']);

% Add parameter source constants for the function block
add_block('simulink/Sources/Constant', [model_name '/Const_m_body'], 'Value', 'm_body', 'Position', [900, 150, 940, 170]);
add_block('simulink/Sources/Constant', [model_name '/Const_L'], 'Value', 'L', 'Position', [900, 180, 940, 200]);
add_block('simulink/Sources/Constant', [model_name '/Const_g'], 'Value', 'g', 'Position', [900, 210, 940, 230]);
add_block('simulink/Sources/Constant', [model_name '/Const_J'], 'Value', 'J_pivot', 'Position', [900, 240, 940, 260]);

% Integrators for Theta (Pitch)
add_block('simulink/Continuous/Integrator', [model_name '/Integ_Theta_Dot'], ...
    'InitialCondition', '0.0', 'Position', [1120, 95, 1150, 125]);
add_block('simulink/Continuous/Integrator', [model_name '/Integ_Theta'], ...
    'InitialCondition', 'init_pitch', 'Position', [1200, 95, 1230, 125]);

% Integrators for X Position
add_block('simulink/Continuous/Integrator', [model_name '/Integ_Pos_X'], ...
    'InitialCondition', '0.0', 'Position', [900, 30, 930, 60]);

% Scopes
add_block('simulink/Sinks/Scope', [model_name '/Scope_Pitch'], 'Position', [1300, 95, 1330, 125]);
add_block('simulink/Sinks/Scope', [model_name '/Scope_Position'], 'Position', [1000, 30, 1030, 60]);

%% 5. Connect Signal Lines
% Target X and Pos X into outer loop
add_line(model_name, 'Target_X/1', 'Sum_Pos/2');
add_line(model_name, 'Integ_Pos_X/1', 'Sum_Pos/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Pos/1', 'Gain_Kp_Pos/1');
add_line(model_name, 'Motor_Lag/1', 'Gain_Kd_Pos/1', 'autorouting', 'on');

add_line(model_name, 'Gain_Kp_Pos/1', 'Sum_Outer/1');
add_line(model_name, 'Gain_Kd_Pos/1', 'Sum_Outer/2');
add_line(model_name, 'Sum_Outer/1', 'Invert_Target_Pitch/1');
add_line(model_name, 'Invert_Target_Pitch/1', 'Sat_Target_Pitch/1');

% Target Pitch into Pitch Error Sum
add_line(model_name, 'Sat_Target_Pitch/1', 'Sum_Pitch/2');
add_line(model_name, 'Integ_Theta/1', 'Sum_Pitch/1', 'autorouting', 'on');

% Pitch Error into PID
add_line(model_name, 'Sum_Pitch/1', 'Gain_Kp_Pitch/1');
add_line(model_name, 'Sum_Pitch/1', 'Integrator_Pitch_Err/1');
add_line(model_name, 'Integrator_Pitch_Err/1', 'Gain_Ki_Pitch/1');
add_line(model_name, 'Integ_Theta_Dot/1', 'Gain_Kd_Pitch/1', 'autorouting', 'on');

add_line(model_name, 'Gain_Kp_Pitch/1', 'Sum_Inner/1');
add_line(model_name, 'Gain_Ki_Pitch/1', 'Sum_Inner/2');
add_line(model_name, 'Gain_Kd_Pitch/1', 'Sum_Inner/3');
add_line(model_name, 'Sum_Inner/1', 'Sat_Velocity/1');

% Velocity to Plant
add_line(model_name, 'Sat_Velocity/1', 'Motor_Lag/1');
add_line(model_name, 'Motor_Lag/1', 'Integ_Pos_X/1', 'autorouting', 'on');
add_line(model_name, 'Motor_Lag/1', 'Accel_Deriv/1');

% MATLAB Function inputs
add_line(model_name, 'Integ_Theta/1', 'Pendulum_Dynamics/1', 'autorouting', 'on');
add_line(model_name, 'Integ_Theta_Dot/1', 'Pendulum_Dynamics/2', 'autorouting', 'on');
add_line(model_name, 'Accel_Deriv/1', 'Pendulum_Dynamics/3');
add_line(model_name, 'Const_m_body/1', 'Pendulum_Dynamics/4');
add_line(model_name, 'Const_L/1', 'Pendulum_Dynamics/5');
add_line(model_name, 'Const_g/1', 'Pendulum_Dynamics/6');
add_line(model_name, 'Const_J/1', 'Pendulum_Dynamics/7');

% Dynamics output to integrators
add_line(model_name, 'Pendulum_Dynamics/1', 'Integ_Theta_Dot/1');
add_line(model_name, 'Integ_Theta_Dot/1', 'Integ_Theta/1');

% Scopes
add_line(model_name, 'Integ_Theta/1', 'Scope_Pitch/1');
add_line(model_name, 'Integ_Pos_X/1', 'Scope_Position/1');

% Save model
save_system(model_name);
fprintf('Simulink model "%s.slx" created and saved successfully!\n', model_name);
