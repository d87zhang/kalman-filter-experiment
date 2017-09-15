function s_hat = toyArm(z, measurement_sigma)
    global dt t_f NUM_ITER n m;    
    
    % dynamic parameters
    link1_m = 3.7;
    link1_COM_x = -0.2;
    link1_inertia_about_z = link1_m * link1_COM_x^2;
    link2_m = 8.2;
    link2_COM_x = -1.1;
    link2_inertia_about_z = link2_m * link2_COM_x^2;
    % initial guesses
    % TODO trivial case
    ig_link1_m = link1_m;
    ig_link1_COM_x = link1_COM_x;
    ig_link1_inertia_about_z = link1_inertia_about_z;
    ig_link2_m = link2_m;
    ig_link2_COM_x = link2_COM_x;
    ig_link2_inertia_about_z = link2_inertia_about_z;
    
    % TODO This fucks things up
%     ig_link1_m = 3;
%     ig_link1_COM_x = -0.3;
%     ig_link1_inertia_about_z = ig_link1_m * ig_link1_COM_x^2;
%     ig_link2_m = 8.7;
%     ig_link2_COM_x = -0.7;
%     ig_link2_inertia_about_z = ig_link2_m * ig_link2_COM_x^2;

    s_actual = [link1_m, link1_COM_x, link1_inertia_about_z, ...
                link2_m, link2_COM_x, link2_inertia_about_z];
    robot = buildPlaneMan(s_actual);

    %% specifications
    % s_k = [m1, com_x1, inertia_about_z1, m2, com_x2, inertia_about_z2];
    % z_k = [t1, t2];

    %% Simulation related stuff
    torque = repmat( sin(linspace(0, 2*pi, NUM_ITER))', 1, m );

    % initial state
    q = zeros(NUM_ITER, m);
    qd = zeros(NUM_ITER, m);
    qdd = zeros(NUM_ITER, m);

    for iter = 2:NUM_ITER
        q_now = q(iter-1, :);
        qd_now = qd(iter-1, :);
        t_now = (iter - 1) * dt;
        [q_end, qd_end] = forwardDynamics(robot, t_now, q_now, qd_now, torque(iter-1, :), dt);
        q(iter, :) = q_end;
        qd(iter, :) = qd_end;
        % calculate qdd myself..
        qdd(iter, :) = (qd_end - qd_now)/dt;
    end
    disp('simulation done!');

    %% TODO testing
%     robot.plot(q);

    % each state is assumed to be independent of each other, so Q is diagonal.
    % similar for R.
    % TODO tune these to see what happens. Create a function around these
    Q = diag(repmat(0.1, n, 1)); % keeping these constant for now
    R = diag(repmat(measurement_sigma^2, m, 1));

    %% Estimation!
    % initialize and initial guesses
    s_hat = zeros(NUM_ITER, n);         % a posteri estimate of s
    P = zeros(n, n, NUM_ITER);          % a posteri error estimate
    s_hat_minus = zeros(NUM_ITER, n);   % a priori estimate of s
    P_minus = zeros(n, n, NUM_ITER);    % a priori error estimate
    K = zeros(n, m, NUM_ITER);          % gain or blending factor

    s_hat(1, :) = [ig_link1_m, ig_link1_COM_x, ig_link1_inertia_about_z, ...
                   ig_link2_m, ig_link2_COM_x, ig_link2_inertia_about_z];
    P(:, :, 1) = eye(n, n);
    
    % small difference in states used for numerical differentiation wrt s
    ds = 0.005 * ones(n, 1);
    % calculate Jacobian matrices that stay constant...
    A = eye(n);
    W = eye(n);
    V = eye(m);
    for k = 2:NUM_ITER
        % calculate non-constant Jacobian matrices numerically
        H_k = computeH(s_hat_minus(k, :), q(k,:), qd(k,:), qdd(k,:));
        
        % time update
        s_hat_minus(k, :) = s_hat(k-1, :); % dynamic parameters are not expected to change
        P_minus(:,:,k) = A*P(:,:,k-1)*A' + W*Q*W';

        % measurement update
%         K(:,:,k) = P_minus(:,:,k) * (H_k'\( H_k*P_minus(:,:,k)*H_k' + V*R*V' ));
        K(:,:,k) = P_minus(:,:,k) * H_k' * inv(H_k*P_minus(:,:,k)*H_k' + V*R*V');
        z_tilde_k = inverseDynamics(buildPlaneMan(s_hat_minus(k,:)), q(k,:), qd(k,:), qdd(k,:));
        s_hat(k,:) = s_hat_minus(k) + K(:,:,k) * (z(k,:)' - z_tilde_k);
        P(:,:,k) = (eye(n) - K(:,:,k) * H_k) * P_minus(:,:,k);
        
        disp(strcat('done iteration ', num2str(k)));
    end
    
    %% Plot results!
    hold on
    % Plot of torque measurements
    t = linspace(0, t_f, NUM_ITER);
    plot(t, torque(:,1), 'DisplayName', 'actual torque 1');
    plot(t, torque(:,2), 'DisplayName', 'actual torque 2');
    scatter(t, z(:,1), 'X', 'DisplayName', 'noisy torque 1 meas');
    scatter(t, z(:,2), 'O', 'DisplayName', 'noisy torque 2 meas');
    
    title('Torque vs. time');
    xlabel('Time(s)');
    ylabel('Torque(whatever torque is usually in)');
    
    % Plot of mass estimates
    figure; hold on
    plot([0, t_f], s_actual(1) * ones(1, 2), 'DisplayName', 'actual mass 1');
    plot([0, t_f], s_actual(4) * ones(1, 2), 'DisplayName', 'actual mass 2');
    plot(t, s_hat(:,1), 'DisplayName', 'mass 1 est.');
    plot(t, s_hat(:,4), 'DisplayName', 'mass 2 est.');
    
    title('Mass est vs. time');
    xlabel('Time(s)');
    ylabel('mass(kg)');
    ylim([min(link1_m, link2_m) - 10, max(link1_m, link2_m) + 10]);
    
    % Plot of COM x pos estimates
    figure; hold on
    plot([0, t_f], s_actual(2) * ones(1, 2), 'DisplayName', 'actual COM x 1');
    plot([0, t_f], s_actual(5) * ones(1, 2), 'DisplayName', 'actual COM x 2');
    plot(t, s_hat(:,2), 'DisplayName', 'COM x 1 est.');
    plot(t, s_hat(:,5), 'DisplayName', 'COM x 2 est.');
    
    title('COM x est vs. time');
    xlabel('Time(s)');
    ylabel('COM x(m)');
    ylim([min(link1_COM_x, link2_COM_x) - 3, max(link1_COM_x, link2_COM_x) + 3]);
    
    % Plot of moment of inertia estimates
    figure; hold on
    plot([0, t_f], s_actual(3) * ones(1, 2), 'DisplayName', 'actual mass 1');
    plot([0, t_f], s_actual(6) * ones(1, 2), 'DisplayName', 'actual mass 2');
    plot(t, s_hat(:,3), 'DisplayName', 'mass 1 est.');
    plot(t, s_hat(:,6), 'DisplayName', 'mass 2 est.');
    
    title('Moment of inertia est vs. time');
    xlabel('Time(s)');
    ylabel('Moment of inertia(whatever torque is usually in)');
    ylim([min(link1_inertia_about_z, link2_inertia_about_z) - 10, ...
          max(link1_inertia_about_z, link2_inertia_about_z) + 10]);
    
    %% helper functions
    function H = computeH(s_hat_minus_k, q_now, qd_now, qdd_now)
        % Computes the Jacobian matrix H numerically
        baseRobo = buildPlaneMan(s_hat_minus_k);
        baseTorque = inverseDynamics(baseRobo, q_now, qd_now, qdd_now);
        
        H = zeros(m, n);
        % vary each state separately to calculate each column of H
        for s_idx = 1:n
            s_deviant = s_hat_minus_k;
            s_deviant(s_idx) = s_deviant(s_idx) + ds(s_idx);
            deviantRobo = buildPlaneMan(s_deviant);
            deviantTorque = inverseDynamics(deviantRobo, q_now, qd_now, qdd_now);
            H(:, s_idx) = (deviantTorque - baseTorque)./ds(s_idx);
        end
    end
    
end % function toyArm