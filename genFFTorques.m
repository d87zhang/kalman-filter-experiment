function torque = genFFTorques(coef, t)
     % Generate finite fourier series torques for testing.
    % coef - m x (2N+1) coefficients matrix, where torques are generated for
    %        m joints and uses N harmonics. Each row's first element is the
    %        constant offset and every pair of elements after that are like
    %        the principle amplitude or something..
    % t - vector of times that we're interested in
    
    assert(mod(size(coef, 2), 2) == 1);
    N = (size(coef, 2) - 1) / 2; % num of harmonics
    m = size(coef, 1);
    NUM_ITER = length(t);
    
    T_f = 1.2; % fundamental period
    w_f = 2*pi/T_f; % fundamental pulsation
    
    torque = repmat(coef(:,end)', NUM_ITER, 1);
    for joint_idx = 1:m
        for harmonic_idx = 1 : N
            a = coef(joint_idx, 2*(harmonic_idx-1) + 1);
            b = coef(joint_idx, 2*(harmonic_idx-1) + 2);
            
            torque(:,joint_idx) = torque(:,joint_idx) ...
                + a/(w_f*harmonic_idx) * sin(w_f * harmonic_idx * t) ...
                - b/(w_f*harmonic_idx) * cos(w_f * harmonic_idx * t);
        end
    end
end