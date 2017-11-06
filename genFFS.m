function [out, out_d, out_dd] = genFFS(coef, t, fund_periods, t_offsets)
    % Generate finite fourier series output.
    % coef - m x (2N+1) coefficients matrix, where torques are generated for
    %        m joints and uses N harmonics. Each row's first element is the
    %        constant offset and every pair of elements after that are like
    %        the principle amplitude or something..
    % t - vector of times that we're interested in
    % fund_periods - fundamental periods vector, one for each joint
    % t_offsets - time offsets vector, one for each joint
    
    assert(mod(size(coef, 2), 2) == 1);
    N = (size(coef, 2) - 1) / 2; % num of harmonics
    m = size(coef, 1);
    NUM_ITER = length(t);
    
    out = repmat(coef(:,end)', NUM_ITER, 1); % apply constant offset
    out_d = zeros(size(out));
    out_dd = out_d;
    for joint_idx = 1:m
        w_f = 2*pi/fund_periods(joint_idx); % fundamental pulsation
        
        for harmonic_idx = 1 : N
            a = coef(joint_idx, 2*(harmonic_idx-1) + 1);
            b = coef(joint_idx, 2*(harmonic_idx-1) + 2);
            
            out(:,joint_idx) = out(:,joint_idx) ...
                + a/(w_f*harmonic_idx) * sin(w_f * harmonic_idx * (t+t_offsets(joint_idx))) ...
                - b/(w_f*harmonic_idx) * cos(w_f * harmonic_idx * (t+t_offsets(joint_idx)));
            
            out_d(:,joint_idx) = out_d(:,joint_idx) ...
                + a * cos(w_f * harmonic_idx * (t+t_offsets(joint_idx))) ...
                + b * sin(w_f * harmonic_idx * (t+t_offsets(joint_idx)));
            
            out_dd(:,joint_idx) = out_dd(:,joint_idx) ...
                - a*(w_f*harmonic_idx) * sin(w_f * harmonic_idx * (t+t_offsets(joint_idx))) ...
                + b*(w_f*harmonic_idx) * cos(w_f * harmonic_idx * (t+t_offsets(joint_idx)));
        end
    end
end