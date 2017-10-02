function torque = genTorques(coef, t)
    % [TODO delete this function] Generate torques for testing.
    % coef - m x p coefficients matrix, where torques are generated for m
    %        joints and uses p/3 sinusoids of the form
    %        a * sin(2*pi/b * t + c). (p must be divisible by 3)
    % t - vector of times that we're interested in
    
    assert(mod(size(coef, 2), 3) == 0);
    num_sin = size(coef, 2) / 3;
    m = size(coef, 1);
    NUM_ITER = length(t);
    
    torque = zeros(NUM_ITER, m);
    for joint_idx = 1:m
        for sin_idx = 0 : num_sin-1
            a = coef(joint_idx, 3*sin_idx+1);
            b = coef(joint_idx, 3*sin_idx+2); % the period
            c = coef(joint_idx, 3*sin_idx+3);
            
            torque(:,joint_idx) = torque(:,joint_idx) ...
                + a * sin(2*pi/b * t + c);
        end
    end
end