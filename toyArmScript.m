%% initialize
dt = 0.005;
t_f = 10;
NUM_ITER = t_f / dt + 1;
t = linspace(0, t_f, NUM_ITER)';
n = 3 * 10; % dimension of s
% n = 2;
m = 3; % dimension of z
NUM_JOINTS = 6;

assumed_measurement_sigma = 1 * [2, 3, 1.5];
% assumed_measurement_sigma = 1 * [6, 4];
% assumed_measurement_sigma = 1 * [6];
measurement_sigma = assumed_measurement_sigma;
% measurement_sigma = zeros(1, m);

% build robo
robot_build_func = @buildPumaDH;
robot_set_params_func = @setPumaParams;
robot_set_param_func = @setPumaParam;
% dynamic parameters
s_actual = zeros(n, 1);
% for modified DH convention Puma
% s_actual(1:10) = [0, 0, 0, 0, 0   0   0.35    0   0   0];
% s_actual(11:20) = [17.4, 0.068, 0.006, -0.016, ...
%                   .13   .524    .539    0     0   0];
% s_actual(21:30) = [4.8, 0, -0.070, 0.014, ...
%                   .066    .0125   .066    0   0   0];
% for DH convention Puma
s_actual(1:10) = [0, 0, 0, 0, 0   0.35   0   0   0   0];
s_actual(11:20) = [17.4, -0.3638, 0.006, 0.2275, ...
                  0.13, 0.524, 0.539, 0, 0, 0];
s_actual(21:30) = [4.8, -0.0203, -0.0141, 0.070, ...
                  0.066, 0.086, 0.0125, 0, 0, 0];
              
% s_actual = [1.5, 2];

% build robo (but it fake)
% robot_build_func = @(s)(FakeRobo(s, NUM_JOINTS));
% s_actual = [2.4, 6, 3.7, 7.3, 4];

robot = robot_build_func(s_actual);

% this is so important :)
[sf1_y, sf1_Fs] = audioread('soundeffects\militarycreation.wav');
[sf2_y, sf2_Fs] = audioread('soundeffects\villagercreation.wav');

%% trajectory gen and simulate robot
coef_file = matfile('coef.mat');
coef = coef_file.ff_coef2;
% fund_periods = [6     4     5     7     3     8];
fund_periods = 5 * ones(1, NUM_JOINTS);
% coef = coef_file.ff_coef_plane;
t_offsets = [1.4, -0.8, 0.7, 1.2 0.3 -2.1];

[q, qd, qdd] = genFFS(coef, t, fund_periods, t_offsets);

% generate quintic splines
% rng(666);
% t_sites = 0:2:t_f;
% NUM_SITES = length(t_sites);
% 
% MAX_Y = 2.5;
% MAX_YD = 6;
% MAX_YDD = 20;
% q_spec = MAX_Y*rand(NUM_SITES, NUM_JOINTS) - MAX_Y/2;
% qd_spec = MAX_YD*rand(NUM_SITES, NUM_JOINTS) - MAX_YD/2;
% qdd_spec = MAX_YDD*rand(NUM_SITES, NUM_JOINTS) - MAX_YDD/2;

% modify boundary condition
% [~, t_begin_idx] = ismember(t_sites(1), t);
% assert(all(t_begin_idx));
% q_spec(1,:) = q(t_begin_idx,:);
% qd_spec(1,:) = qd(t_begin_idx,:);
% qdd_spec(1,:) = qdd(t_begin_idx,:);

% [q, qd, qdd] = quinticSpline(q_spec, qd_spec, qdd_spec, t_sites, t);
% [q_spl, qd_spl, qdd_spl] = quinticSpline(q_spec, qd_spec, qdd_spec, ...
%                                          t_sites, t(t_begin_idx:end));

% q(t_begin_idx:end,:) = q_spl;
% qd(t_begin_idx:end,:) = qd_spl;
% qdd(t_begin_idx:end,:) = qdd_spl;

% Generate required torque for this trajectory
torque = robot.rne([q, qd, qdd], robot.gravity, zeros(1,6));
% [torque, base_wrench] = robot.rne([q, qd, qdd], robot.gravity, zeros(1,6));

% torque = zeros(NUM_ITER, NUM_JOINTS);
% for k = 1:NUM_ITER
%     [temp_torque, base_wrench] = robot.rne([q(k,:), qd(k,:), qdd(k,:)], ...
%                                            robot.gravity, ...
%                                            zeros(1,6));
%     torque(k,:) = temp_torque;
% end

%% generate measurements
z = zeros(NUM_ITER, m);
for idx = 1:m
    z(:,idx) = torque(:,idx) + normrnd(0, measurement_sigma(idx), NUM_ITER, 1);
end

%% estimate parameters
disp('Start estimating!');

P_0 = zeros(1, n);
s_hat_1 = s_actual;

chosen_indices = [6,11,21,15:20,25:30]; % parameters being estimated
guess_factors = containers.Map(chosen_indices, 1.5 * ones(size(chosen_indices)));
% guess_factors(15) = 1.5 * 1.5;

for idx = chosen_indices
    P_0(idx) = 1;
    s_hat_1(idx) = guess_factors(idx) * s_hat_1(idx);

    % auto tuning
%     error_in_guess = s_hat_1(idx) - s_actual(idx);
%     if error_in_guess == 0 
%         P_0(idx) = 1;
%     else
%         P_0(idx) = error_in_guess^2;
%     end
end

% fine tuning P_0...
P_0(6) = 0.16;
P_0(11) = 15;
P_0(15) = 0.3;
P_0(16) = 0.3;
P_0(25) = 0.03;
P_0(26) = 0.03;
P_0(27) = 0.01;

P_0 = diag(P_0);

est_robot = robot_build_func(s_hat_1);

% ds = small difference in states used for numerical differentiation
% ds = max(s_hat_1 * 0.01, 0.001*ones(size(s_hat_1)));
ds = 1e-5 * ones(n, 1);

tic
[s_hat, H, residual, K, P] = ...
    estimateParams(z, assumed_measurement_sigma, P_0, ...
                   q, qd, qdd, s_hat_1, ds, ...
                   est_robot, robot_set_param_func, robot_set_params_func);
toc
disp('Done estimating \[T]/');
if rand() > 0.5
    sound(sf1_y, sf1_Fs);
else
    sound(sf2_y, sf2_Fs);
end

%% Plot results!
folderName = 'C:\Users\Difei\Desktop\toyArm pics\currPlots\';
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

% for 2 DoF Plane Man
% figure('units','normalized','outerposition',[0 0 1 1]); hold on
% plot([0, t_f], s_actual(1) * ones(1, 2), 'DisplayName', 'true mass 1');
% plot([0, t_f], s_actual(2) * ones(1, 2), 'DisplayName', 'true mass 2');
% plot(t, s_hat(:,1), 'DisplayName', 'est mass 1');
% plot(t, s_hat(:,2), 'DisplayName', 'est mass 2');
% 
% title('Estimates vs. time');
% xlabel('Time(s)');
% ylabel('Mass(kg)');
% 
% legend('show');
% saveas(gcf, strcat(folderName, '2-estimates.jpg'));

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
ylabel('Time(s)');
ylabel('something..');

legend('show');
saveas(gcf, strcat(folderName, '6-P norm.jpg'));

% Save textual results
results = zeros(length(chosen_indices), 7);
init_guesses = strings(length(chosen_indices), 1);
final_perc_errs = zeros(1, length(chosen_indices));
mean_abs_rel_err = zeros(1, length(chosen_indices));
for i = 1:length(chosen_indices)
    idx = chosen_indices(i);
    init_guesses(i) = string(sprintf('%0.3f (%0.3f)', ...
                                     s_hat_1(idx), s_hat_1(idx)/s_actual(idx)));
    
    diff = s_hat(:,idx) - s_actual(idx) * ones(size(s_hat(:,idx)));
    normalized_integral = sum(abs(diff)) / s_actual(idx);
    mean_abs_rel_err(i) = normalized_integral / NUM_ITER;
    final_perc_errs(i) = 100*(s_hat(end,idx) - s_actual(idx))/s_actual(idx);
                                    
    results(i,:) = [idx, P_0(idx, idx), s_actual(idx), s_hat(end,idx), ...
                    100*P(idx, idx, end)/P_0(idx, idx), ...
                    100*mean_abs_rel_err(i), ...
                    final_perc_errs(i)];
end
additional_info = zeros(length(chosen_indices), 2);
additional_info(:,1) = results(:,1);
for i = 1:size(additional_info, 1)
    additional_info(i,2) = guess_factors(additional_info(i,1));
end

resultsTable1 = table(results(:,1), results(:,2), results(:,3), ...
    init_guesses, results(:,4), results(:,5), results(:,6), results(:,7));
resultsTable1.Properties.VariableNames = {'stateId', 'P_0_val', ...
    'actualVal', 'initGuess_factor', 'finalEst', 'final_P_val_over_P_0_perc', ...
    'avg_rel_err_perc', 'final_perc_err'};
resultsTable1Str = evalc('disp(resultsTable1)');
% get rid of silly formatting stuff in the string..
resultsTable1Str = regexprep(resultsTable1Str, '(</strong>|<strong>)', '');

resultsFileName = [folderName, 'results.txt'];
resultsFile = fopen(resultsFileName, 'w');
fprintf(resultsFile, resultsTable1Str);

mean_abs_rel_err_without_infs = mean_abs_rel_err(~isinf(mean_abs_rel_err));
final_perc_errs_without_infs = final_perc_errs(~isinf(final_perc_errs));
fprintf(resultsFile, 'Over all estimated parameters (not %%, ignores Inf terms):\n');
fprintf(resultsFile, 'mean (absolute value) relative error: %f\n', ...
        mean(mean_abs_rel_err_without_infs));
fprintf(resultsFile, 'mean of relative error squared: %f\n', ...
        mean(mean_abs_rel_err_without_infs.^2));
fprintf(resultsFile, 'mean (absolute value) final relative error: %f\n', ...
        mean(abs(final_perc_errs_without_infs)) / 100);
fprintf(resultsFile, 'mean of final relative error squared: %f\n', ...
        mean((final_perc_errs_without_infs/100).^2));
    
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

