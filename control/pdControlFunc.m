function torque = pdControlFunc(t_now, q_desired, q_now, qd_desired, qd_now, ...
                                Kp, Kd)
    % Returns control output (torque) as a row vector
    % Kp - proportional gain vector
    % Kd - differential gain vector
    Kp = diag(Kp);
    Kd = diag(Kd);
    torque = (q_desired - q_now) * Kp + (qd_desired - qd_now) * Kd;
end