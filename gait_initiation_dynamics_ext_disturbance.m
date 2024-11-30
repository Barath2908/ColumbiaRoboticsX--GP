function dydt = gait_initiation_dynamics_ext_disturbance(t, y, params)
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

    % PID Controller Output
    T_PID = Kp * error + Ki * integral + Kd * derivative_of_error;

    % Feedforward Torque
    T_F_current = params.T_F(t);

    % **External Disturbance Torque**
    T_disturbance_current = params.T_disturbance(t);

    % Total Ankle Torque (No Saturation)
    T_Ank = T_F_current + T_PID;

    % Dynamic Equation with Disturbance
    theta_ddot = (T_Ank + T_disturbance_current - b * theta_dot - (k + m_B * g * L_COM) * theta) / J_B;

    % Update state derivatives
    dydt = zeros(3,1);
    dydt(1) = theta_dot;           % d(theta)/dt
    dydt(2) = theta_ddot;          % d(theta_dot)/dt
    dydt(3) = error;               % d(integral)/dt = error
end
