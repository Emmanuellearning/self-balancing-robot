%% =========================================================================
% Pure Control Theory: Inverted Pendulum Inner PID Balancing Loop
% Generates a clean 4-block closed-loop SISO system in Simulink
% =========================================================================
clear; clc;

%% 1. Plant Physical Parameters
L = 0.20;     % Effective length / COM height (m)
g = 9.81;     % Gravity (m/s^2)

% Transfer Function: P(s) = (1/L) / (s^2 - g/L)
num_plant = [1/L];              % 5
den_plant = [1, 0, -g/L];        % s^2 - 49.05

%% 2. PID Controller Gains
Kp = 22.0;    % Proportional gain (stiffness)
Ki = 0.4;     % Integral gain (eliminates steady-state error)
Kd = 2.5;     % Derivative gain (damping / pulls poles to LHP)

%% 3. Build Theoretical Simulink Model
model_name = 'theory_inner_loop';

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
new_system(model_name);
open_system(model_name);

set_param(model_name, 'StopTime', '3.0');
set_param(model_name, 'Solver', 'ode45');

%% 4. Add the 4 Core Blocks
% 1. Step Reference Input (Command 0.05 rad ~ 2.8 deg tilt at t = 0.5s)
add_block('simulink/Sources/Step', [model_name '/Theta_Target'], ...
    'Time', '0.5', 'Before', '0', 'After', '0.05', ...
    'Position', [60, 100, 100, 130]);

% 2. Error Sum: e(s) = Theta_Target - Theta
add_block('simulink/Math Operations/Sum', [model_name '/Sum_Error'], ...
    'Inputs', '+-', ...
    'Position', [160, 105, 180, 135]);

% 3. Standard PID Controller Block
add_block('simulink/Continuous/PID Controller', [model_name '/PID_Controller'], ...
    'P', 'Kp', 'I', 'Ki', 'D', 'Kd', ...
    'Position', [240, 95, 300, 145]);

% 4. Plant Transfer Function: P(s) = (1/L) / (s^2 - g/L)
add_block('simulink/Continuous/Transfer Fcn', [model_name '/Plant_P(s)'], ...
    'Numerator', 'num_plant', 'Denominator', 'den_plant', ...
    'Position', [370, 95, 480, 145]);

% 5. Scope to view response
add_block('simulink/Sinks/Scope', [model_name '/Scope_Theta'], ...
    'Position', [550, 105, 580, 135]);

%% 5. Connect Signal Lines
% Target -> Error Sum
add_line(model_name, 'Theta_Target/1', 'Sum_Error/1');

% Error -> PID Controller
add_line(model_name, 'Sum_Error/1', 'PID_Controller/1');

% PID Controller -> Plant
add_line(model_name, 'PID_Controller/1', 'Plant_P(s)/1');

% Plant -> Scope
add_line(model_name, 'Plant_P(s)/1', 'Scope_Theta/1');

% Plant -> Negative Feedback to Sum
add_line(model_name, 'Plant_P(s)/1', 'Sum_Error/2', 'autorouting', 'on');

%% 6. Save and Finish
save_system(model_name);
fprintf('Theoretical inner-loop model "%s.slx" created successfully!\n', model_name);
