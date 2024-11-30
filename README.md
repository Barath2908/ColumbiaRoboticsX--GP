# Columbia-Robotics-Group-Project
-------------------------------------------
# Gait Initiation Model with Disturbance Rejection

This repository contains MATLAB/Simulink implementations of an inverted pendulum model for gait initiation, extending the work of Petrucci et al. (2018) with additional disturbance rejection capabilities.

## Overview

The model simulates anticipatory postural adjustments (APAs) during gait initiation, with particular focus on:
- Basic gait initiation dynamics
- Reduced dorsiflexor torque effects (as seen in Parkinson's disease)
- Internal disturbance handling
- External disturbance rejection

## Model Components

### Core Model Structure
- Single-link inverted pendulum in sagittal plane
- Body mass (mB): 70 kg
- Body length (LB): 1.8 m
- Center of mass height (L_COM): 0.9 m
- Moment of inertia (JB): 75.6 kg⋅m²

### Controller Design
1. PID Controller
   - Proportional gain (Kp): 100
   - Integral gain (Ki): 200
   - Derivative gain (Kd): 300
   
2. Feed-forward Component
   ```matlab
   T_F = J_B * theta_ref_ddot - m_B * g * L_COM * theta_ref
   ```

3. Sensory Feedback
   - Time delay: 0.171s
   - Position and velocity feedback

## Disturbance Models

### Internal Disturbances
1. Tremors
   - Oscillatory disturbance
   - Amplitude: 1.0 N⋅m
   - Frequency: 5 Hz
   ```matlab
   T_tremor = A_tremor * sin(omega_tremor * t)
   ```

2. Rigidity
   - Increased joint stiffness
   - Factor: 2.0x normal stiffness

3. Bradykinesia
   - Reduced movement speed
   - Scale: 0.7 of normal controller output

4. Postural Instability
   - Sudden perturbations
   - Amplitude: 10 N⋅m
   - Occurs every 2 seconds
   - Duration: 0.1s

### External Disturbances
- Sinusoidal disturbance
- Amplitude: 10 N⋅m
- Frequency: 0.5 Hz
```matlab
T_disturbance = A * sin(omega * t)
```

## File Structure

1. `gait_initiation_animation.m`
   - Basic model implementation
   - Animation of inverted pendulum
   - PID control visualization

2. `gait_initiation_dynamics.m`
   - Core dynamics equations
   - State-space model implementation

3. `gait_initiation_animation_i.m`
   - Added model with internal disturbances
   - Visualization of disturbance effects

4. `gait_initiation_dynamics_i.m`
   - Modified dynamics including internal disturbances
   - Enhanced controller implementation

5. `gait_initiation_ext_disturbance_animation.m`
   - Added Model with external disturbance implementation
   - Extended visualization capabilities

6. `gait_initiation_dynamics_ext_disturbance.m`
   - Dynamics with external disturbance handling
   - Combined control strategy

7. `SLIP_params.m`
   - System parameters definition
   - Physical constants
   - Controller gains

# Detailed MATLAB Files Documentation

## 1. gait_initiation_animation.m
Main simulation file implementing the basic model.

### Key Components:
- System Parameters:
  ```matlab
  J_B = 75.6;              % Moment of inertia (kg·m²)
  b = 1;                   % Damping coefficient (N·m·s/rad)
  k = 1;                   % Spring constant (N·m/rad)
  m_B = 70;                % Body mass (kg)
  L_COM = 0.9;             % Distance to center of mass (m)
  ```

- PID Controller Parameters:
  ```matlab
  Kp = 100;                % Proportional gain
  Ki = 200;                % Integral gain
  Kd = 300;                % Derivative gain
  ```

- Simulation Settings:
  ```matlab
  t_start = 0;             % Start time (s)
  t_end = 250;             % End time (s)
  dt = 0.001;              % Time step (s)
  theta_desired = 0.35;    % Desired lean angle (rad)
  ```

- Features:
  * Basic inverted pendulum simulation
  * Real-time animation
  * Performance plots (lean angle, torque, error)
  * No disturbance handling

## 2. gait_initiation_dynamics.m
Implements core system dynamics.

### Key Functions:
```matlab
function dydt = gait_initiation_dynamics(t, y, params)
    % State variables
    theta = y(1);          % Lean angle
    theta_dot = y(2);      % Angular velocity
    integral = y(3);       % Integral of error
    prev_error = y(4);     % Previous error
```

- Handles:
  * Basic state equations
  * PID control implementation
  * Feedforward torque calculation
  * Error computation

## 3. gait_initiation_animation_i.m
Extended model with internal disturbances.

### Additional Features:
- Internal Disturbance Parameters:
  ```matlab
  % Tremors
  A_tremor = 1.0;          % Amplitude (N·m)
  f_tremor = 5;            % Frequency (Hz)
  
  % Rigidity
  rigidity_factor = 2.0;   % Increased stiffness
  
  % Bradykinesia
  bradykinesia_scale = 0.7; % Movement reduction
  
  % Postural Instability
  A_instability = 10;      % Amplitude (N·m)
  occurrence_interval = 2;  % Every 2 seconds
  ```

- Visualization:
  * Multiple disturbance effects
  * Combined torque plots
  * Disturbance-specific animations

## 4. gait_initiation_dynamics_i.m
Handles dynamics with internal disturbances.

### Main Functions:
```matlab
function dydt = gait_initiation_dynamics_i(t, y, params)
    % Additional components for internal disturbances
    T_tremor_current = params.T_tremor(t);
    T_instability_current = params.T_instability(t);
    k_eff = k * params.rigidity_factor;
```

- Features:
  * Modified state equations
  * Disturbance integration
  * Enhanced control strategy

## 5. gait_initiation_ext_disturbance_animation.m
Implements external disturbance handling.

### Key Components:
- External Disturbance Parameters:
  ```matlab
  A = 10;                  % Amplitude (N·m)
  f = 0.5;                 % Frequency (Hz)
  omega = 2 * pi * f;      % Angular frequency
  
  T_disturbance = @(t) A * sin(omega * t);
  ```

- Enhanced Visualization:
  * External disturbance effects
  * Combined system response
  * Modified animation sequence

## 6. gait_initiation_dynamics_ext_disturbance.m
Handles dynamics with external disturbances.

### Core Functions:
```matlab
function dydt = gait_initiation_dynamics_ext_disturbance(t, y, params)
    % External disturbance integration
    T_disturbance_current = params.T_disturbance(t);
    
    % Modified dynamic equation
    theta_ddot = (T_Ank + T_disturbance_current - b * theta_dot 
                 - (k + m_B * g * L_COM) * theta) / J_B;
```

## 7. SLIP_params.m
System parameters and constants.

### Parameter Definitions:
```matlab
% Physical parameters
mB = 70;          % Body mass (kg)
LB = 1.8;         % Body length (m)
L_COM = LB / 2;   % COM distance (m)
JB = (1/3) * mB * LB^2;  % Moment of inertia

% Control limits
T_DF_max = -22;   % Max dorsiflexor torque
TSAT = 8;         % Saturation value

% Simulation parameters
simulation_time = 2; % Duration (s)
```

### Usage:
- Central parameter repository
- Consistent values across simulations
- Easy modification of system properties

## Key Interfaces Between Files:
1. Parameter passing through params structure
2. Common dynamics calculations
3. Shared visualization components
4. Unified disturbance handling

## Running Simulations:
1. Load parameters: `SLIP_params.m`
2. Choose simulation type:
   - Basic: `gait_initiation_animation.m`
   - Internal disturbances: `gait_initiation_animation_i.m`
   - External disturbances: `gait_initiation_ext_disturbance_animation.m`
3. View results in generated plots and animations

## Usage

1. Basic Model:
```matlab
run('gait_initiation_animation.m')
```

2. Internal Disturbances:
```matlab
run('gait_initiation_animation_i.m')
```

3. External Disturbances:
```matlab
run('gait_initiation_ext_disturbance_animation.m')
```

## Results

The model demonstrates:
1. Stable tracking of desired lean angle
2. Effective disturbance rejection
3. Realistic simulation of Parkinson's disease effects
4. Robust performance under various perturbations

## References

1. Petrucci, M. N., DiBerardino, L. A., MacKinnon, C. D., & Hsiao-Wecksler, E. T. (2018). A neuromechanical model of reduced dorsiflexor torque during the anticipatory postural adjustments of gait initiation. IEEE Transactions on Neural Systems and Rehabilitation Engineering, 26(11), 2210-2216.
