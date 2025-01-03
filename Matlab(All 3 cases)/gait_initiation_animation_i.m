% gait_initiation_simulation_with_animation.m
% Simulates gait initiation with internal disturbances and animates the results

clear; close all; clc;

% System Parameters
J_B = 75.6;              % Moment of inertia (kg·m²)
b = 1;                   % Damping coefficient (N·m·s/rad)
k = 1;                   % Spring constant (N·m/rad)
m_B = 70;                % Body mass (kg)
g = 9.81;                % Acceleration due to gravity (m/s²)
L_COM = 0.9;             % Distance to center of mass (m)

% PID Controller Gains
Kp = 100;                % Proportional gain
Ki = 200;                % Integral gain
Kd = 300;                % Derivative gain

% Simulation Parameters
t_start = 0;             % Start time (s)
t_end = 25;              % End time (s)
dt = 0.001;              % Time step (s)

% Desired Lean Angle (radians)
theta_desired = 0.1;     % Approximately 5.73 degrees

% Define theta_ref as a step input
theta_ref = @(t) theta_desired * (t >= 0.5);  % Step at t = 0.5 s

% Feedforward Torque Calculation
% For step input, theta_ref_ddot = 0 after the step
T_F = @(t) -m_B * g * L_COM * sin(theta_ref(t));  % T_F(t) = -m_B * g * L_COM * theta_ref(t)

% -----------------------------
% Internal Disturbances
% -----------------------------

% A. Tremors: Oscillatory Disturbances
A_tremor = 1.0;          % Amplitude in N·m
f_tremor = 5;            % Frequency in Hz
omega_tremor = 2 * pi * f_tremor;
T_tremor = @(t) A_tremor * sin(omega_tremor * t);

% B. Rigidity: Increased Joint Stiffness
rigidity_factor = 2.0;   % Increased stiffness by a factor of 2

% C. Bradykinesia: Delayed and Reduced Movement
bradykinesia_scale = 0.7;   % 70% of normal controller output

% D. Postural Instability: Sudden Perturbations
A_instability = 10;          % Amplitude in N·m
occurrence_interval = 2;     % Occurs every 2 seconds
duration_instability = 0.1;  % Duration of each instability event (s)
instability_times = t_start:occurrence_interval:t_end;

T_instability = @(t) sum(A_instability * ((t >= instability_times) & (t < instability_times + duration_instability)));

% -----------------------------
% Define parameters as a structure
% -----------------------------
params.J_B = J_B;
params.b = b;
params.k = k;
params.m_B = m_B;
params.g = g;
params.L_COM = L_COM;
params.Kp = Kp;
params.Ki = Ki;
params.Kd = Kd;
params.theta_ref = theta_ref;
params.T_F = T_F;
params.dt = dt;
params.t_start = t_start;
params.t_end = t_end;

% Add internal disturbances to parameters
params.T_tremor = T_tremor;
params.rigidity_factor = rigidity_factor;
params.bradykinesia_scale = bradykinesia_scale;
params.T_instability = T_instability;

% Define the ODE function handle
ode_func = @(t, y) gait_initiation_dynamics_i(t, y, params);

% Run the simulation using ode45
options = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t, y] = ode45(ode_func, [t_start t_end], [0; 0; 0], options);

% Extract results
theta = y(:,1);
theta_dot = y(:,2);
integral = y(:,3);

% Calculate theta_ref over time for plotting
theta_ref_values = arrayfun(theta_ref, t);

% Calculate error
error = theta_ref_values - theta;

% Calculate T_PID and T_F for plotting
derivative_of_error = -theta_dot;  % Since d(theta_ref)/dt ≈ 0
T_PID_unscaled = Kp * error + Ki * integral + Kd * derivative_of_error;  % Unscaled PID output
T_PID = bradykinesia_scale * T_PID_unscaled;  % Scaled PID output due to bradykinesia
T_F_values = arrayfun(T_F, t);                % T_F(t) over time
T_tremor_values = arrayfun(T_tremor, t);      % Tremor torque over time
T_instability_values = arrayfun(T_instability, t);  % Instability torque over time

% Effective stiffness over time (constant in this case)
k_eff = k * rigidity_factor;

% Calculate Total Ankle Torque
T_total = T_F_values + T_PID + T_tremor_values;

% Calculate Net Torque (including disturbances)
Net_Torque = T_total + T_instability_values;

% Plot Simulation Results
figure;
subplot(4,1,1);
plot(t, theta_ref_values, 'r--', 'LineWidth', 2);
hold on;
plot(t, theta, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Lean Angle (rad)');
legend('\theta_{ref}', '\theta_{model}');
title('Lean Angle Tracking');
grid on;

subplot(4,1,2);
plot(t, T_PID, 'g-', 'LineWidth', 2);
hold on;
plot(t, T_F_values, 'm--', 'LineWidth', 2);
plot(t, T_tremor_values, 'c-', 'LineWidth', 1);
plot(t, T_instability_values, 'r-', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Torque (N·m)');
legend('T_{PID}', 'T_F', 'T_{tremor}', 'T_{instability}');
title('Torque Components');
grid on;

subplot(4,1,3);
plot(t, T_total, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Ankle Torque (N·m)');
legend('T_{Ank}');
title('Total Ankle Torque');
grid on;

subplot(4,1,4);
plot(t, error, 'k-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Error (rad)');
legend('Error');
title('Tracking Error');
grid on;

% -----------------------------
% Animation Section
% -----------------------------

% Physical Parameters for Animation
body_length = 1;        % Length of the pendulum (meters)
pivot_point = [0, 0];   % Ankle joint at origin
scale = 1;              % Scaling factor for visualization

% Initialize Figure for Animation
figure('Name', 'Gait Initiation Animation', 'NumberTitle', 'off');
axis equal;
grid on;
hold on;

% Define Axis Limits
xlim([-1.5 * body_length, 1.5 * body_length]);
ylim([-0.1, 1.5 * body_length]);
xlabel('Horizontal Position (m)');
ylabel('Vertical Position (m)');
title('Gait Initiation Animation');

% Plot Ground Line
plot([-1.5 * body_length, 1.5 * body_length], [0, 0], 'k-', 'LineWidth', 2);

% Initialize Pendulum (Body) as a Line
pendulum_line = plot([pivot_point(1), pivot_point(1) + body_length * sin(theta(1))], ...
                    [pivot_point(2), pivot_point(2) + body_length * cos(theta(1))], ...
                    'b-', 'LineWidth', 4);

% Initialize Reference Angle Indicator (Optional)
ref_line = plot([pivot_point(1), pivot_point(1) + body_length * sin(theta_ref_values(1))], ...
               [pivot_point(2), pivot_point(2) + body_length * cos(theta_ref_values(1))], ...
               'r--', 'LineWidth', 2);

% Add Ankle Marker
ankle_marker = plot(pivot_point(1), pivot_point(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% Add Center of Mass Marker
com_position = @(theta_current) pivot_point + 0.5 * body_length * [sin(theta_current), cos(theta_current)];
com_current = com_position(theta(1));
com_marker = plot(com_current(1), com_current(2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

% Animation Speed Control
animation_speed = 1;    % 1 for real-time, >1 for faster, <1 for slower

% Loop Through Each Time Step for Animation
for i = 1:length(t)
    % Current Lean Angle
    current_theta = theta(i);

    % Calculate Pendulum End Position
    pendulum_end = pivot_point + body_length * [sin(current_theta), cos(current_theta)];

    % Update Pendulum Line
    set(pendulum_line, 'XData', [pivot_point(1), pendulum_end(1)], ...
                      'YData', [pivot_point(2), pendulum_end(2)]);

    % Update Center of Mass Position
    com_current = com_position(current_theta);
    set(com_marker, 'XData', com_current(1), 'YData', com_current(2));

    % Update Reference Angle Indicator
    ref_end = pivot_point + body_length * [sin(theta_ref_values(i)), cos(theta_ref_values(i))];
    set(ref_line, 'XData', [pivot_point(1), ref_end(1)], ...
                 'YData', [pivot_point(2), ref_end(2)]);

    % Render the Frame
    drawnow;

    % Control Animation Speed
    if i < length(t)
        pause((t(i+1) - t(i)) / animation_speed);
    end
end
