% Test how correlation differs between trajectories - a simple case
% Requires the robot to be ready as well as initial setup for estimation
% (e.g. chosen_indices, P_0, s_hat_1)

assert(length(chosen_indices) == 2);

num_trajs = 4;
a_array = cell(1, num_trajs);
b_array = cell(1, num_trajs);

a_array(1) = {[1, 1.5]};
b_array(1) = {[0, 0]};
a_array(2) = {[1.5, -0.5]};
b_array(2) = {[2, 1]};
a_array(3) = {[-2, -0.75]};
b_array(3) = {[-1, 0]};
a_array(4) = {[0.5, -1]};
b_array(4) = {[0.5, -1]};

corr_per_traj = zeros(NUM_ITER, num_trajs);
i = chosen_indices(1); % param indices
j = chosen_indices(2);
for traj_idx = 1:num_trajs
    a = a_array{traj_idx};
    b = b_array{traj_idx};
    [ q, qd, qdd ] = linearTraj(a, b, t);
    
    % generic stuff (wrt traj_idx)
    torque = robot.rne([q, qd, qdd], robot.gravity, zeros(1,6));
    genMeas;
    doTheEstimation;
    calcCorr;
    
    corr_per_traj(:,traj_idx) = reshape(corr(i,j,:), ...
                                        size(corr_per_traj(:,traj_idx)));
end

% plot results
figure('units','normalized','outerposition',[0 0 1 1]); hold on;

for traj_idx = 1:num_trajs
    plot(t, corr_per_traj(:,traj_idx), 'DisplayName', sprintf('traj #%d', traj_idx));
end

legend('show');
title(sprintf('Corr values vs time for params (%d, %d), CoM estimated alone: %s', ...
      i, j, BOOL_TO_STRING{EST_CENTER_OF_MASS_ALONE + 1}));
ylim([-1, 1]);

saveas(gcf, strcat(tempFolderName, 'Corr values vs time over all trajs.jpg'));
