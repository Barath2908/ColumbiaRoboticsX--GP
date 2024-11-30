% Physical parameters
mB = 70;          % Body mass without feet (kg)
LB = 1.8;         % Body length (m)
L_COM = LB / 2;   % Distance to center of mass (m)
JB = (1/3) * mB * LB^2;  % Moment of inertia about the ankle (kg·m^2)
g = 9.81;         % Acceleration due to gravity (m/s^2)

% Spring and damper parameters
k = 1;          % Spring stiffness (Nm/rad)
b = 1;           % Damping coefficient (Nm·s/rad)

% Time delay
Td = 0.171;       % Time delay (s)

% Torque saturation
T_DF_max = -22;   % Maximum dorsiflexor torque (Nm)
TSAT = 8;         % Saturation value (Nm)

% Simulation parameters
simulation_time = 2; % Simulation duration (s)

% Reference trajectory
% Load or define theta_ref as a function of time
% For simplicity, let's define it as a constant or a predefined function
