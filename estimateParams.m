function [s_hat, H, residual] = estimateParams(z, assumed_measurement_sigma, Q, ...
                                     q, qd, qdd, s_hat_1, ds, robotBuildFunc)
    NUM_ITER = size(z, 1);
    m = size(z, 2);
    n = length(s_hat_1);

    R = diag(repmat(assumed_measurement_sigma^2, m, 1));

    %% Estimation!
    % initialize and initial guesses
    s_hat = zeros(NUM_ITER, n);         % a posteri estimate of s
    P = zeros(n, n, NUM_ITER);          % a posteri error estimate
    s_hat_minus = zeros(NUM_ITER, n);   % a priori estimate of s
    P_minus = zeros(n, n, NUM_ITER);    % a priori error estimate
    K = zeros(n, m, NUM_ITER);          % gain or blending factor

    s_hat(1, :) = s_hat_1;
    P(:, :, 1) = eye(n, n);
    
    % calculate Jacobian matrices that stay constant...
    A = eye(n);
    W = eye(n);
    V = eye(m);
    H = zeros(m, n, NUM_ITER);
    
    % for plotting purposes
    residual = zeros(NUM_ITER, m);
    for k = 2:NUM_ITER
        % time update
        s_hat_minus(k, :) = s_hat(k-1, :); % dynamic parameters are not expected to change
        P_minus(:,:,k) = A*P(:,:,k-1)*A' + W*Q(:,:,k-1)*W';

        % calculate non-constant Jacobian matrices numerically
        H(:,:,k) = computeH(s_hat_minus(k, :), q(k,:), qd(k,:), qdd(k,:));
        
        % measurement update
%         K(:,:,k) = P_minus(:,:,k) * (H(:,:,k)'\( H(:,:,k)*P_minus(:,:,k)*H(:,:,k)' + V*R*V' ));
        K(:,:,k) = P_minus(:,:,k) * H(:,:,k)' ...
                   * inv(H(:,:,k)*P_minus(:,:,k)*H(:,:,k)' + V*R*V');
        z_tilde_k = inverseDynamics(robotBuildFunc(s_hat_minus(k,:)), q(k,:), qd(k,:), qdd(k,:));
        z_tilde_k = z_tilde_k(1:m);
        residual(k,:) = z(k,:) - z_tilde_k';
        s_hat(k,:) = s_hat_minus(k,:) + ( K(:,:,k) * residual(k,:)' )';
        P(:,:,k) = (eye(n) - K(:,:,k) * H(:,:,k)) * P_minus(:,:,k);
        
%         disp(strcat('done iteration ', num2str(k)));
    end
    
    %% helper functions
    function H = computeH(s_hat_minus_k, q_now, qd_now, qdd_now)
        % Computes the Jacobian matrix H numerically
        baseRobo = robotBuildFunc(s_hat_minus_k);
        baseTorque = inverseDynamics(baseRobo, q_now, qd_now, qdd_now);
        
        H = zeros(m, n);
        % vary each state separately to calculate each column of H
        for s_idx = 1:n
            s_deviant = s_hat_minus_k;
            s_deviant(s_idx) = s_deviant(s_idx) + ds(s_idx);
            deviantRobo = robotBuildFunc(s_deviant);
            deviantTorque = inverseDynamics(deviantRobo, q_now, qd_now, qdd_now);
            H(:, s_idx) = (deviantTorque(1:m) - baseTorque(1:m))./ds(s_idx);
        end
        
    end
    
end