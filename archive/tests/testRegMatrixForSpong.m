%% set-up

% test regMatrixForSpongPlaneMan() with calcThetaForSpongPlaneMan() by
% comparing torques generated by them against torques stored in the
% variable `torque` (presumably generated by more venerable methods)
%
% Requires `q`, `qd`, `qdd`, `torque` and `s_actual` to be populated. See
% main.m for how to get these

Y = regMatrixForSpongPlaneMan(q, qd, qdd);
% calls calcThetaForSpongPlaneMan() through SPONG_PLANE_MAN_SETUP.theta_func
theta = SPONG_PLANE_MAN_SETUP.theta_func(s_actual); % minimal parameter set

torque_by_reg = zeros(size(torque));
for k = 1:NUM_ITER
    torque_by_reg(k,:) = (Y(:,:,k) * theta)';
end

%% show results

figure('units','normalized','outerposition',[0 0 1 1]); hold on;
for idx = 1:NUM_JOINTS
    plot(t, torque(:,idx), 'DisplayName', sprintf('rne torque %d', idx));
    plot(t, torque_by_reg(:,idx), 'DisplayName', sprintf('regressor + theta torque %d', idx));
end

xlabel('Time(s)');
ylabel('Torque(N*m)');
legend('show');

%% test by asserting (actually there is a non-trivial difference, so this would fail)
% assert( all(all( abs(torque_by_reg - torque) < 1e-3)) );
