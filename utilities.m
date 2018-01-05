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
xlabel('Time(s)');
ylabel('Observability matrix rank');
legend('show');

saveas(gcf, strcat(folderName, 'Observability matrix rank.fig')); % TODO remove these saveas() calls

%% save init conditions
save('temp_saved_vars\init_cond.mat', 's_hat_1', '-append');
save('temp_saved_vars\init_cond.mat', 'P_0', '-append');

%% load init conditions
init_cond_file = matfile('temp_saved_vars\init_cond.mat');
s_hat_1 = init_cond_file.s_hat_1;
P_0 = init_cond_file.P_0;

%% detect parameters that don't contribute in H
H_threshold = 1e-5;

none_contributors_mask = false(1, n);
for idx = 1:n
    none_contributors_mask(idx) = all(all(H(:,idx,:) < H_threshold));
end
list_params = 1:n;
none_contributors_idx = list_params(none_contributors_mask);

%% plotting variances (on-diagonal P values)
figure('units','normalized','outerposition',[0 0 1 1]); hold on;

for idx = chosen_indices
    plot(t, reshape(P(idx, idx, :), 1, size(P, 3)), 'DisplayName', getParamDescript(idx));
end

legend('show');
xlabel('Time(s)');
ylabel('Variance');
title('Variances vs time');

saveas(gcf, strcat(tempFolderName, 'On diagonal P values.fig'));

%% plotting part of H
figure('units','normalized','outerposition',[0 0 1 1]); hold on;

% param_ids_of_interest = chosen_indices;
param_ids_of_interest = [7];

for i = 1:m
    for j = param_ids_of_interest
        plot(t, reshape(H(i,j,:), 1, size(H, 3)), ...
            'DisplayName', sprintf('H(%d,%d)', i, j));
    end
end

legend('show');
title('Some H values');

saveas(gcf, strcat(tempFolderName, 'Some H values.jpg'));

%% Plot corr value
figure('units','normalized','outerposition',[0 0 1 1]); hold on;

for i = chosen_indices
    for j = chosen_indices
        % only examine pairs where j > i
        if j <= i
            continue;
        end
        plot(t, reshape(corr(i,j,:), 1, size(corr, 3)), ...
            'DisplayName', sprintf('corr(%d,%d)', i, j));
    end
end

legend('show');
title('Corr values vs time');
ylim([-1, 1]);

saveas(gcf, strcat(tempFolderName, 'Corr values vs time.jpg'));

%% imagesc corr matrix
% take a certain percentile for clim
iter = 2000;

corr_mat = corr(:,:,iter);

figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(corr_mat, [-1, 1]);
colorbar;
title(sprintf('Correlation matrix at iteration %d', iter));

%% plot a parameter estimate along with a confidence interval
figure('units','normalized','outerposition',[0 0 1 1]); hold on;

param_idx = 1;

interval_radius = reshape(P(param_idx,param_idx,:), numel(P(param_idx,param_idx,:)), 1).^(1/2);
ciplot(s_hat(:,param_idx) - interval_radius, s_hat(:,param_idx) + interval_radius, t, 'm', ...
              'DisplayName', 'confidence interval');
plot(t, s_hat(:,param_idx), 'DisplayName', 'estimate');
plot([0, t_f], s_actual(param_idx) * ones(1, 2), 'DisplayName', 'true value');

title(['Estimate and confidence interval of ', getParamDescript(param_idx)]);
legend('show');
xlabel('Time(s)');