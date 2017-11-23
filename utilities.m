%% Generate FF trajectory coefficients
num_joints = 2;
num_harmonics = 5;
% these amplitudes will be fudged, so they are not the exact values
% mean_amplitudes = 0.5 * [2.5, 1.8, 2.5, 2.5, 2.5, 2.5]';
mean_amplitudes = 0.5 * [1.6, 2.2]';
% mean_offsets = [1, 1, 1, 0.5, 0.5, 0.5]';
mean_offsets = [1, 1]';
mean_offsets = ((rand(num_joints,1) > 0.5)*2 - 1) .* mean_offsets; % randomly flip signs

ff_coef_plane = zeros(num_joints, 2*num_harmonics + 1);
% offsets
ff_coef_plane(:,end) = normrnd(1, 0.15, num_joints, 1) .* mean_offsets;
% amplitudes
mean_amplitudes = repmat(mean_amplitudes, 1, 2*num_harmonics);
ff_coef_plane(:,1:end-1) = normrnd(1, 0.15, num_joints, 2*num_harmonics) .* mean_amplitudes;

save('coef.mat', 'ff_coef_plane', '-append');

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

%% Plot of observability matrix's rank
% rankFunc = @rank;
rankFunc = @(A)(myRank(A, 1e7));

Ob_rank_t_invar = zeros(1, NUM_ITER); % assuming a time-invariant system
Ob_rank_t_var = zeros(1, NUM_ITER); % assuming a time-varying system
for k = 1:NUM_ITER
    % assuming a time-invariant system
    Ob = obsv(eye(n), H(:,:,k));
    Ob_rank_t_invar(k) = rankFunc(Ob);
    
    % assuming a time-varying system
    Ob = myObsv(H, k);
    Ob_rank_t_var(k) = rankFunc(Ob);
end

figure('units','normalized','outerposition',[0 0 1 1]); hold on;
plot([0, t_f], n * ones(1,2), 'DisplayName', 'number of state variables');
plot(t, Ob_rank_t_invar, 'DisplayName', 'Ob matrix rank (time-invariant)');
plot(t, Ob_rank_t_var, 'DisplayName', 'Ob matrix rank (time-varying)');
title('Observability matrix (time-invariant) rank vs time');
ylabel('Time(s)');
ylabel('Observability matrix rank');
legend('show');

saveas(gcf, strcat(folderName, 'Observability matrix rank.fig'));