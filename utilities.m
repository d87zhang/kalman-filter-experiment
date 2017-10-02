%% Generate FF torque coefficients
num_torques = 6;
num_harmonics = 5;
% these amplitudes will be fudged, so they are not the exact values
mean_amplitudes = 1 * [100, 100, 80, 5, 5, 5]';
mean_offsets = [20, 20, 15, 2, 2, 2]';
mean_offsets = ((rand(num_torques,1) > 0.5)*2 - 1) .* mean_offsets; % randomly flip signs

ff_coef = zeros(num_torques, 2*num_harmonics + 1);
% offsets
ff_coef(:,end) = normrnd(1, 0.15, num_torques, 1) .* mean_offsets;
% amplitudes
mean_amplitudes = repmat(mean_amplitudes, 1, 2*num_harmonics);
ff_coef(:,1:end-1) = normrnd(1, 0.15, num_torques, 2*num_harmonics) .* mean_amplitudes;

save('coef.mat', 'ff_coef', '-append');

%% see how far torque predictions differ..
s_hat_file = matfile('s_hat_vanilla.mat');
s_hat_test = s_hat_file.s_hat;

s_est = s_hat_test(end,:);
robot_est = buildPuma(s_est);

torque_est = zeros(NUM_ITER, size(torque, 2));
for k = 1:NUM_ITER
    torque_est(k,:) = inverseDynamics(robot_est, q(k,:), qd(k,:), qdd(k,:));
end
residual_est = torque_est - torque;

figure; hold on
for idx = 1:size(residual_est, 2)
    plot(t, residual_est(:,idx), 'DisplayName', sprintf('residual for torque %d', idx));
end

title('Residual vs. time');
xlabel('Time(s)');
ylabel('Residual torque(N*m)');

legend('show');
