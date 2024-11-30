function dydt = gait_initiation_dynamics_i(t, y, params)
    % gait_initiation_dynamics defines the ODEs for the gait initiation model with internal disturbances
    %
    % Inputs:
    %   t - Current time
    %   y - State vector [theta; theta_dot; integral]
    %   params - Structure containing system parameters and controller gains
    %
    % Outputs:
    %   dydt - Derivatives of the state vector

    % Unpack parameters
    J_B = params.J_B;
    b = params.b;
    k = params.k;
    m_B = params.m_B;
    g = params.g;
    L_COM = params.L_COM;
    Kp = params.Kp;
    Ki = params.Ki;
    Kd = params.Kd;
    theta_ref = params.theta_ref;

    % State variables
    theta = y(1);          % Lean angle
    theta_dot = y(2);      % Angular velocity
    integral = y(3);       % Integral of error

    % Calculate desired lean angle
    theta_ref_current = theta_ref(t);

    % Calculate error
    error = theta_ref_current - theta;

    % Compute derivative of error
    derivative_of_error = -theta_dot;  % Assuming d(theta_ref)/dt ≈ 0

    % PID Controller Output with Bradykinesia Scaling
    T_PID_unscaled = Kp * error + Ki * integral + Kd * derivative_of_error;
    T_PID = params.bradykinesia_scale * T_PID_unscaled;

    % Feedforward Torque
    T_F_current = params.T_F(t);

    % External Disturbance Torque
    T_disturbance_current = 0;  % Assuming no external disturbances here

    % Tremor Torque
    T_tremor_current = params.T_tremor(t);

    % Instability Torque
    T_instability_current = params.T_instability(t);

    % Total Ankle Torque including tremor
    T_Ank = T_F_current + T_PID + T_tremor_current;

    % Effective Stiffness with Rigidity
    k_eff = k * params.rigidity_factor;

    % Dynamic Equation with Disturbances
    theta_ddot = (T_Ank + T_instability_current - b * theta_dot - (k_eff + m_B * g * L_COM) * theta) / J_B;

    % Update state derivatives
    dydt = zeros(3,1);
    dydt(1) = theta_dot;           % d(theta)/dt
    dydt(2) = theta_ddot;          % d(theta_dot)/dt
    dydt(3) = error;               % d(integral)/dt = error
end
