%% initialize
dt = 0.005;
t_f = 10;
NUM_ITER = t_f / dt + 1;
t = linspace(0, t_f, NUM_ITER)';

% rng seeds
guess_rng_seed = 123;
s_actual_rng_seed = 321;
quintic_traj_rng_seed = 666;

% setups, see some defined in util/initSetups.m
curr_setup = SPONG_PLANE_MAN_SETUP;
% curr_setup = SIMPLE_PLANE_MAN_SETUP;
% curr_setup = DH_PUMA_SETUP;

n = curr_setup.n; % dimension of s
m = curr_setup.m; % dimension of z
NUM_JOINTS = curr_setup.NUM_JOINTS;
robot_build_func = curr_setup.robot_build_func;
robot_set_params_func = curr_setup.robot_set_params_func;
robot_set_param_func = curr_setup.robot_set_param_func;
assumed_measurement_sigma = curr_setup.assumed_measurement_sigma;
s_actual = curr_setup.s_actual;
traj_coef = curr_setup.traj_coef;

% test only measuring a subset of torques
% m = 1;
assumed_measurement_sigma = assumed_measurement_sigma(1:m);

% measurement_sigma = assumed_measurement_sigma;
measurement_sigma = zeros(1, m);

% Set up initial conditions (initial guess and P_0)
P_0 = zeros(1, n);
s_hat_1 = zeros(size(s_actual));

% chosen_indices = 1:n; % parameters being estimated
chosen_indices = [1, 2, 7, 11, 12, 17]; % parameters being estimated

guess_factors = containers.Map(chosen_indices, 1.5 * ones(size(chosen_indices)));
% randomize guess_factors a bit, with bounds
rng(guess_rng_seed);
for idx = chosen_indices
    rand_range = 0.6;
    if rand() > 0.5
        % turn over-estimate to under-estimate and vice versa
        guess_factors(idx) = 1 - (guess_factors(idx) - 1);
    end
    guess_factors(idx) = guess_factors(idx) + rand_range * rand() - rand_range/2;
end
% guess_factors(1) = 2;
% guess_factors(2) = 2;

for idx = chosen_indices
    P_0(idx) = 1;

    % hardcoding
    P_0(idx) = 10;
    
    % auto tuning
%     if s_actual(idx) == 0 
%         P_0(idx) = 1;
%     else
%         P_0(idx) = (s_hat_1(idx) - s_actual(idx))^2;
%     end
end

% fine tuning P_0...
% P_0(1) = 1;
% P_0(2) = 0.25;

P_0 = diag(P_0);

Q = 2 * 1e-3 * P_0./P_0;
Q(isnan(Q)) = 0; % get rid of NaN from dividing by 0
% Q = zeros(size(P_0));

% randomly perturb s_actual
% rng(s_actual_rng_seed);
% for i = 1:numel(chosen_indices)
%     idx = chosen_indices(i);
%     perturb_factor = 10^(2 * rand() - 1);
%     s_actual(idx) = s_actual(idx) * perturb_factor;
%     
%     % automatically compensate by scaling P_0 and Q
%     P_0(idx,idx) = P_0(idx,idx) * perturb_factor^2;
%     Q(idx,idx) = Q(idx,idx) * perturb_factor^2;
% end

robot = robot_build_func(s_actual);

% form initial guesses
for idx = chosen_indices
    s_hat_1(idx) = guess_factors(idx) * s_actual(idx);
end

%% trajectory gen and simulate robot
% TODO make sure everything in this section (including called functions are
% well documented)

% % fund_periods = [6     4     5     7     3     8];
% fund_periods = 5 * ones(1, NUM_JOINTS);
% t_offsets = [1.4, -0.8, 0.7, 1.2 0.3 -2.1];
% 
% [q, qd, qdd] = genFFS(traj_coef, t, fund_periods, t_offsets);

% q = repmat(q(2.5 * 200:3 * 200,:),50,1);
% q = q(1:numel(t),:);
% qd = repmat(qd(2.5 * 200:3 * 200,:),50,1);
% qd = qd(1:numel(t),:);
% qdd = repmat(qdd(2.5 * 200:3 * 200,:),50,1);
% qdd = qdd(1:numel(t),:);

% ==============
% Some kind of special traj
% a1 = [1, 1.5];
% b1 = [0, 0];
% a2 = [1.5, -0.5];
% b2 = [2, 1];
% a3 = [-2, -0.75];
% b3 = [-1, 0];
% a4 = [0.5, -1];
% b4 = [0.5, -1];
% 
% [ q2, qd2, qdd2 ] = linearTraj(a1, b1, t);
% 
% cat_point = round(length(t) /2);
% q(cat_point:end, :) = q2(cat_point:end, :);
% qd(cat_point:end, :) = qd2(cat_point:end, :);
% qdd(cat_point:end, :) = qdd2(cat_point:end, :);

% % q(:,2) = zeros(size(q(:,2)));
% q(:,2) = pi * ones(size(q(:,2)));
% qd(:,2) = zeros(size(qd(:,2)));
% qdd(:,2) = zeros(size(qdd(:,2)));
% 
% % Add a component to q1 and its derivatives in order to up qdd1 by a
% % constant amount, so that it stays above or below zero
% % First scale down the existing trajectory though so the required constant
% % gain is smaller.
% scale_facotr = 1/8;
% q(:,1) = scale_facotr * q(:,1);
% qd(:,1) = scale_facotr * qd(:,1);
% qdd(:,1) = scale_facotr * qdd(:,1);
% 
% qdd1_constant_gain = -20 * scale_facotr;
% q(:,1) = q(:,1) + qdd1_constant_gain * 0.5 * t.^2;
% qd(:,1) = qd(:,1) + qdd1_constant_gain * t;
% qdd(:,1) = qdd(:,1) + qdd1_constant_gain;
% ==============

% generate quintic splines
rng(quintic_traj_rng_seed);
t_sites = 0:2:t_f;
NUM_SITES = length(t_sites);

MAX_Y = 2.5;
MAX_YD = 6;
MAX_YDD = 20;
q_spec = MAX_Y*rand(NUM_SITES, NUM_JOINTS) - MAX_Y/2;
qd_spec = MAX_YD*rand(NUM_SITES, NUM_JOINTS) - MAX_YD/2;
qdd_spec = MAX_YDD*rand(NUM_SITES, NUM_JOINTS) - MAX_YDD/2;

% modify boundary condition
% [~, t_begin_idx] = ismember(t_sites(1), t);
% assert(all(t_begin_idx));
% q_spec(1,:) = q(t_begin_idx,:);
% qd_spec(1,:) = qd(t_begin_idx,:);
% qdd_spec(1,:) = qdd(t_begin_idx,:);

[q, qd, qdd] = quinticSpline(q_spec, qd_spec, qdd_spec, t_sites, t);
% [q_spl, qd_spl, qdd_spl] = quinticSpline(q_spec, qd_spec, qdd_spec, ...
%                                          t_sites, t(t_begin_idx:end));

% q(t_begin_idx:end,:) = q_spl;
% qd(t_begin_idx:end,:) = qd_spl;
% qdd(t_begin_idx:end,:) = qdd_spl;

%% generate torques and measurements
% Generate required torque for this trajectory

% [torque, base_wrench] = robot.rne([q, qd, qdd], robot.gravity, zeros(1,6));

% torque = zeros(NUM_ITER, NUM_JOINTS);
% for k = 1:NUM_ITER
%     [temp_torque, base_wrench] = robot.rne([q(k,:), qd(k,:), qdd(k,:)], ...
%                                            robot.gravity, ...
%                                            zeros(1,6));
%     torque(k,:) = temp_torque;
% end
torque = robot.rne([q, qd, qdd], robot.gravity, zeros(1,6));
rng(65535);
genMeas;

%% estimate parameters
doTheEstimation;
calcCorr;

%% Plot results!
% These folders must be created before hand
folderName = 'C:\Users\Difei\Desktop\toyArm pics\currPlots\'; % TODO put this somewhere else or something
tempFolderName = 'C:\Users\Difei\Desktop\toyArm pics\tempPlots\';
YLIM_FACTOR = 3;

% Plot of H's condition number
H_cond = zeros(NUM_ITER, 1);
for k = 1:NUM_ITER
    H_cond(k) = cond(H(:,:,k));
end

figure('units','normalized','outerposition',[0 0 1 1]);
plot(t, H_cond, 'DisplayName', 'Hs condition num', 'color', 'r');

title('Hs condition number vs. time');
xlabel('Time(s)');
ylabel('condition number');
legend('show');
saveas(gcf, strcat(folderName, '1-H condition number.jpg'));

OFFSET_DESCRIPTION_MAP = containers.Map({2, 3, 4}, {'x', 'y', 'z'});

% sanity check - variance should always be positive
for k = 1:NUM_ITER
    if ~all(diag(P(:,:,k)) > -1e-5)
        warning('Some of P''s on-diagonal values are negative at iter = %d', k);
        assert(false);
    end
end

if mod(n, 10) == 0
    for idx = 1:(n/10)
        base_idx = 10 * (idx-1);
        figure('units','normalized','outerposition',[0 0 1 1]);
        % plot of mass estimates
        subplot(3, 1, 1);
        hold on
        plot([0, t_f], s_actual(base_idx + 1) * ones(1, 2), 'color', 'b');
        plot(t, s_hat(:,base_idx + 1), 'color', 'r');
        hold off

        title(sprintf('link %d - mass est vs. time', idx));
        xlabel('Time(s)');
        ylabel('mass(kg)');
        max_abs = max([abs(s_actual(base_idx + 1)); 0.1]);
        ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);

        % plot of first moment estimates
        subplot(3, 1, 2);
        for offset_idx = 2:4
            hold on
            plot([0, t_f], s_actual(base_idx + offset_idx) * ones(1, 2), ...
                'DisplayName', sprintf('true moment - %s', OFFSET_DESCRIPTION_MAP(offset_idx)));
            plot(t, s_hat(:,base_idx + offset_idx), ...
                'DisplayName', sprintf('est moment - %s', OFFSET_DESCRIPTION_MAP(offset_idx)));
            hold off
        end

        title(sprintf('link %d - center of mass est vs. time', idx));
        xlabel('Time(s)');
        ylabel('center of mass(m)');
        max_abs = max([abs(s_actual(base_idx+2:base_idx+4)); 0.1]);
        ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);
        legend('show');

        % plot of moment of inertia estimates
        subplot(3, 1, 3);
        for offset_idx = 5:10
            hold on
            plot([0, t_f], s_actual(base_idx + offset_idx) * ones(1, 2), 'color', 'b');
            plot(t, s_hat(:,base_idx + offset_idx), 'color', 'r');
            hold off
        end

        title(sprintf('link %d - moment of inertia est vs. time', idx));
        xlabel('Time(s)');
        ylabel('Moment of inertia(kg*m^2)');
        max_abs = max([abs(s_actual(base_idx+5:base_idx+10)); 0.1]);
        ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);

        saveas(gcf, strcat(folderName, sprintf('%d-estimate joint %d.jpg', idx+1, idx)));
    end
else
    % for 2 DoF Plane Man (and occasionally other stuff..)
    figure('units','normalized','outerposition',[0 0 1 1]); hold on
    plot([0, t_f], s_actual(1) * ones(1, 2), 'DisplayName', sprintf('true %s 1', S_SPEC));
    plot([0, t_f], s_actual(2) * ones(1, 2), 'DisplayName', sprintf('true %s 2', S_SPEC));
    plot(t, s_hat(:,1), 'DisplayName', sprintf('est %s 1', S_SPEC));
    plot(t, s_hat(:,2), 'DisplayName', sprintf('est %s 2', S_SPEC));

    title('Estimates vs. time');
    xlabel('Time(s)');
    ylabel('Mass(kg)');

    legend('show');
    saveas(gcf, strcat(folderName, '2-estimates.jpg'));

end

% Plot of residual
figure('units','normalized','outerposition',[0 0 1 1]); hold on
for idx = 1:size(residual, 2)
    plot(t, residual(:,idx), 'DisplayName', sprintf('residual for torque %d', idx));
end

title('Residual vs. time');
xlabel('Time(s)');
ylabel('Residual torque(N*m)');

legend('show');
saveas(gcf, strcat(folderName, '5-residual.jpg'));

% Plot of P's norm
figure('units','normalized','outerposition',[0 0 1 1]);
P_norm = zeros(1, NUM_ITER);
for k = 1:NUM_ITER
    P_norm(k) = norm(P(:,:,k));
end

plot(t, P_norm, 'DisplayName', 'Ps norm');

title('Ps norm vs time');
xlabel('Time(s)');
ylabel('something..');

legend('show');
saveas(gcf, strcat(folderName, '6-P norm.jpg'));

% plot error on minimal set (theta)
if isequal(curr_setup, SPONG_PLANE_MAN_SETUP)
    theta_actual = curr_setup.theta_func(s_actual);
    theta_est = zeros(NUM_ITER, length(theta_actual));
    theta_rel_err = zeros(NUM_ITER, length(theta_actual));
    for k = 1:NUM_ITER
        theta_est(k,:) = curr_setup.theta_func(s_hat(k,:))';
        theta_rel_err(k,:) = (theta_est(k,:) - theta_actual') ./ abs(theta_actual');
    end
    
    figure('units','normalized','outerposition',[0 0 1 1]); hold on
    for theta_idx = 1:length(theta_actual)
        plot(t, theta_rel_err(:, theta_idx), 'DisplayName', sprintf('theta(%d)', theta_idx));
    end
    title('Theta relative error vs time');
    xlabel('Time(s)');
    ylabel('Theta values(unit varies)');
    legend('show');
    saveas(gcf, strcat(folderName, 'theta relative error.jpg'));
end

%% Generate textual results
% ========================
off_diag_cov_sums = reshape(sum(abs(P), 1), size(P, 2), size(P, 3));
for k = 1:NUM_ITER
    off_diag_cov_sums(:,k) = off_diag_cov_sums(:,k) - abs(diag(P(:,:,k)));
end

results = zeros(length(chosen_indices), 8);
final_perc_errs = zeros(1, length(chosen_indices));
mean_abs_rel_err = zeros(1, length(chosen_indices));
sum_abs_corr = zeros(1, n);

for i = 1:length(chosen_indices)
    idx = chosen_indices(i);
    
    diff = s_hat(:,idx) - s_actual(idx) * ones(size(s_hat(:,idx)));
    normalized_integral = abs(sum(abs(diff)) / s_actual(idx));
    mean_abs_rel_err(i) = normalized_integral / NUM_ITER;
    final_perc_errs(i) = 100*(s_hat(end,idx) - s_actual(idx))/abs(s_actual(idx));
    
    results(i,:) = [idx, ... % 1
                    P_0(idx, idx), ... % 2
                    s_actual(idx), ... % 3
                    s_hat(end,idx), ... % 4
                    100*P(idx, idx, end)/P_0(idx, idx), ... % 5
                    100*mean_abs_rel_err(i), ... % 6
                    final_perc_errs(i), ... % 7
                    off_diag_cov_sums(idx,end)]; % 8
end
additional_info = zeros(length(chosen_indices), 4);
additional_info(:,1) = results(:,1);
for i = 1:size(additional_info, 1)
    idx = chosen_indices(i);
    
    P_0_value = diag(P_0);
    P_0_value = P_0_value(idx);
    Q_value = diag(Q);
    Q_value = Q_value(idx);
    additional_info(i,2:end) = [guess_factors(idx), ...
                                P_0_value, ...
                                Q_value];
end

resultsTable1 = table(results(:,1), results(:,3), ...
    results(:,5), results(:,6), results(:,7), results(:,8));
resultsTable1.Properties.VariableNames = {'stateId', ...
    'actualVal', 'final_P_val', ...
    'avg_rel_err_perc', 'final_perc_err', ...
    'final_off_diag_cov_sum'};
additional_info_table = table(additional_info(:,1), additional_info(:,2), ...
                              additional_info(:,3), additional_info(:,4));
additional_info_table.Properties.VariableNames = {'stateId', ...
    'init_guess_factor', 'P_0_value', 'Q_value'};

resultsTable1Str = evalc('disp(resultsTable1)');
additional_info_table_str = evalc('disp(additional_info_table)');
% get rid of silly formatting stuff in the string..
resultsTable1Str = regexprep(resultsTable1Str, '(</strong>|<strong>)', '');
additional_info_table_str = regexprep(additional_info_table_str, '(</strong>|<strong>)', '');

resultsFileName = [folderName, 'results.txt'];
resultsFile = fopen(resultsFileName, 'w');
fprintf(resultsFile, resultsTable1Str);
fprintf(resultsFile, '*final_P_val is presented as percentage of P_0 value.\n');
fprintf(resultsFile, '*final_perc_err is positive when the final estimate is higher than the actual value and vice versa.\n');

mean_abs_rel_err_cleaned = mean_abs_rel_err(~isinf(mean_abs_rel_err) & ~isnan(mean_abs_rel_err));
final_perc_errs_cleaned = final_perc_errs(~isinf(final_perc_errs) & ~isnan(final_perc_errs));
fprintf(resultsFile, '\n');
fprintf(resultsFile, 'Over all estimated parameters (not %%, ignores Inf terms):\n');
fprintf(resultsFile, 'mean (absolute value) relative error: %f\n', ...
        mean(mean_abs_rel_err_cleaned));
fprintf(resultsFile, 'mean (absolute value) final relative error: %f\n', ...
        mean(abs(final_perc_errs_cleaned)) / 100);

fprintf(resultsFile, '\n');
fprintf(resultsFile, additional_info_table_str);
fprintf(resultsFile, '*init_guess_factor expresses the factor (init_guess_val / actual_val).\n');

fprintf(resultsFile, '\n');
fprintf(resultsFile, 'EST_CENTER_OF_MASS_ALONE: %s\n', BOOL_TO_STRING{EST_CENTER_OF_MASS_ALONE + 1});

% how accurately is the minimal param set (theta) estimated for Spong's 
% 2-DoF plane man?
if isequal(curr_setup, SPONG_PLANE_MAN_SETUP)
    theta_actual = curr_setup.theta_func(s_actual);
    theta_est = curr_setup.theta_func(s_hat(end,:));
    
    theta_rel_err = (theta_est - theta_actual) ./ abs(theta_actual);
    theta_mean_rel_err = mean(abs(theta_rel_err));
    theta_names = ['theta1'; 'theta2'; 'theta3'; 'theta4'; 'theta5'];
    
    fprintf(resultsFile, '\n');
    fprintf(resultsFile, 'Error in minimal parameter set theta (not %%):\n');
    theta_rel_err_table = table(theta_names, theta_actual, theta_rel_err);
	theta_rel_err_table_str = evalc('disp(theta_rel_err_table)');
    % get rid of silly formatting stuff in the string..
    theta_rel_err_table_str = regexprep(theta_rel_err_table_str, '(</strong>|<strong>)', '');
    fprintf(resultsFile, theta_rel_err_table_str);
    fprintf(resultsFile, 'mean theta relative error: %f', theta_mean_rel_err);
end

fprintf(resultsFile, '\n');
fprintf(resultsFile, '================= for Tester =================\n');

fprintf(resultsFile, 'guess_rng_seed: %d\n', guess_rng_seed);
fprintf(resultsFile, 's_actual_rng_seed: %d\n', s_actual_rng_seed);
fprintf(resultsFile, 'quintic_traj_rng_seed: %d\n', quintic_traj_rng_seed);

fclose(resultsFile);

%% More plots
% Plot of torques
figure('units','normalized','outerposition',[0 0 1 1]); hold on
for idx = 1:NUM_JOINTS
    plot(t, torque(:,idx), 'DisplayName', sprintf('joint %d', idx));
end

for idx = 1:m
    scatter(t, z(:,idx), 'DisplayName', sprintf('meas for joint %d', idx));
end
meas_sigma_str = strtrim(sprintf('%d ', measurement_sigma));
title(sprintf('Torque vs. time (meas sigma = %s)', meas_sigma_str));
xlabel('Time(s)');
ylabel('Torque(N*m)');
legend('show');
saveas(gcf, strcat(folderName, '7-torques.jpg'));

%Plot of q's
figure('units','normalized','outerposition',[0 0 1 1]); hold on
for idx = 1:NUM_JOINTS
    plot(t, q(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('q vs. time');
xlabel('Time(s)');
ylabel('Joint angle(rad)');
legend('show');
saveas(gcf, strcat(folderName, '8-q.jpg'));

figure('units','normalized','outerposition',[0 0 1 1]); hold on
for idx = 1:NUM_JOINTS
    plot(t, qd(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('qd vs. time');
xlabel('Time(s)');
ylabel('Joint velocity(rad/s)');
legend('show');
saveas(gcf, strcat(folderName, '9-qd.jpg'));

figure('units','normalized','outerposition',[0 0 1 1]); hold on
for idx = 1:NUM_JOINTS
    plot(t, qdd(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('qdd vs. time');
xlabel('Time(s)');
ylabel('Joint angle(rad/s^2)');
legend('show');
saveas(gcf, strcat(folderName, '10-qdd.jpg'));
