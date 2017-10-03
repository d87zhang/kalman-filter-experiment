function torque = pdControlFunc(t_now, q_desired, q_now, qd_desired, qd_now, ...
                                Kp, Kd, NUM_IGNORED_JOINTS)
    % Returns control output (torque) as a row vector
    % Kp - proportional gain vector
    % Kd - differential gain vector
    % NUM_IGNORED_JOINTS - num of joints for which we just output 0 torque
    Kp = diag(Kp);
    Kd = diag(Kd);
    torque = (q_desired - q_now) * Kp + (qd_desired - qd_now) * Kd;
    torque(end-NUM_IGNORED_JOINTS+1 : end) = zeros(1, NUM_IGNORED_JOINTS);
end