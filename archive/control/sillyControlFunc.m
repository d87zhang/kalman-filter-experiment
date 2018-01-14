function torque = sillyControlFunc(t_now, q_desired, q_now, qd_desired, qd_now, coef, NUM_JOINTS)
    % Returns control output (torque) as a row vector
    assert(mod(size(coef, 2), 2) == 1);
    N = (size(coef, 2) - 1) / 2; % num of harmonics
    m = size(coef, 1);
    
    T_f = 1.2; % fundamental period
    w_f = 2*pi/T_f; % fundamental pulsation
    
    torque = zeros(1, NUM_JOINTS);
    torque(1:m) = coef(:,end)';
    for joint_idx = 1:m
        for harmonic_idx = 1 : N
            a = coef(joint_idx, 2*(harmonic_idx-1) + 1);
            b = coef(joint_idx, 2*(harmonic_idx-1) + 2);
            
            torque(joint_idx) = torque(joint_idx) ...
                + a/(w_f*harmonic_idx) * sin(w_f * harmonic_idx * t_now) ...
                - b/(w_f*harmonic_idx) * cos(w_f * harmonic_idx * t_now);
        end
    end
end