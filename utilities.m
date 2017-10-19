%% Generate FF trajectory coefficients
num_joints = 6;
num_harmonics = 5;
% these amplitudes will be fudged, so they are not the exact values
mean_amplitudes = 0.5 * [2.5, 1.8, 2.5, 2.5, 2.5, 2.5]';
mean_offsets = [1, 1, 1, 0.5, 0.5, 0.5]';
mean_offsets = ((rand(num_joints,1) > 0.5)*2 - 1) .* mean_offsets; % randomly flip signs

ff_coef2 = zeros(num_joints, 2*num_harmonics + 1);
% offsets
ff_coef2(:,end) = normrnd(1, 0.15, num_joints, 1) .* mean_offsets;
% amplitudes
mean_amplitudes = repmat(mean_amplitudes, 1, 2*num_harmonics);
ff_coef2(:,1:end-1) = normrnd(1, 0.15, num_joints, 2*num_harmonics) .* mean_amplitudes;

save('coef.mat', 'ff_coef2', '-append');

%% Generate simple FF torque coefficients
num_joints = 3;
num_harmonics = 5;
% these amplitudes will be fudged, so they are not the exact values
mean_amplitudes = 1 * [15, 15, 15]';
mean_offsets = [10, 10, 10]';
mean_offsets = ((rand(num_joints,1) > 0.5)*2 - 1) .* mean_offsets; % randomly flip signs

simple_ff_coef = zeros(num_joints, 2*num_harmonics + 1);
% offsets
simple_ff_coef(:,end) = normrnd(1, 0.15, num_joints, 1) .* mean_offsets;
% amplitudes
mean_amplitudes = repmat(mean_amplitudes, 1, 2*num_harmonics);
simple_ff_coef(:,1:end-1) = normrnd(1, 0.15, num_joints, 2*num_harmonics) .* mean_amplitudes;

save('coef.mat', 'simple_ff_coef', '-append');

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

%% make trajectory flat at the beginning
flat_ending_idx = 2.5/dt + 1;
q(1:flat_ending_idx, :) = repmat(q(flat_ending_idx, :), flat_ending_idx, 1);
qd(1:flat_ending_idx, :) = zeros(size(qd(1:flat_ending_idx, :)));
qdd(1:flat_ending_idx, :) = zeros(size(qdd(1:flat_ending_idx, :)));