%% initialize
dt = 0.001;
t_f = 5;
NUM_ITER = t_f / dt + 1;
t = linspace(0, t_f, NUM_ITER)';
n = 3 * 10; % dimension of s
% n = 3;
m = 3; % dimension of z
NUM_JOINTS = 6;

assumed_measurement_sigma = [10, 10, 3];
measurement_sigma = assumed_measurement_sigma;
% measurement_sigma = zeros(1, m);

% build robo
robot_build_func = @buildPuma;
robot_set_params_func = @setPumaParams;
robot_set_param_func = @setPumaParam;
% dynamic parameters
s_actual = zeros(n, 1);
s_actual(1:10) = [0, 0, 0, 0, 0   0   0.35    0   0   0];
s_actual(11:20) = [17.4, 0.068, 0.006, -0.016, ...
                  .13   .524    .539    0     0   0];
s_actual(21:30) = [4.8, 0, -0.070, 0.014, ...
                  .066    .0125   .066    0   0   0];
% here s contains 3 elements for each link: m, first moment about x, moment
% of inertia about z (zz)
% s_actual = [0, 0, 0.35, ...
%             17.4, 17.4*0.068, .539, ...
%             4.8, 4.8*-0.070, .066];
% s_actual = [0.35, 17.4, 4.8];

% build robo (but it fake)
% robot_build_func = @(s)(FakeRobo(s, NUM_JOINTS));
% s_actual = [2.4, 6, 3.7, 7.3, 4];

robot = robot_build_func(s_actual);

% this is so important :)
[sf1_y, sf1_Fs] = audioread('soundeffects\militarycreation.wav');
[sf2_y, sf2_Fs] = audioread('soundeffects\villagercreation.wav');

%% trajectory gen and simulate robot
coef_file = matfile('coef.mat');
coef = coef_file.ff_coef;
t_offsets = [0.4, -0.8, 0.7, 1.2 0.3 -1.1];
[q, qd, qdd] = genFFS(coef, t, t_offsets);

% % hold trajectory still after a certain time
% t_quintic_0 = 0.5;
% t_hold_still = 1;
% idx_quintic_0 = t_quintic_0/dt + 1;
% idx_hold_still = t_hold_still/dt + 1;
% 
% [ret1, ret2, ret3] = quinticTraj(q_desired(idx_quintic_0,:), ...
%                                  qd_desired(idx_quintic_0,:), ...
%                                  qdd_desired(idx_quintic_0,:), ...
%                                  q_desired(idx_quintic_0,:), ...
%                                  zeros(1, NUM_JOINTS), ...
%                                  zeros(1, NUM_JOINTS), ...
%                                  t(idx_quintic_0:idx_hold_still));
% q_desired(idx_quintic_0:idx_hold_still,:) = ret1;
% qd_desired(idx_quintic_0:idx_hold_still,:) = ret2;
% qdd_desired(idx_quintic_0:idx_hold_still,:) = ret3;
% 
% q_desired(idx_hold_still:end, :) = repmat(q_desired(idx_hold_still, :), ...
%                                           NUM_ITER - idx_hold_still + 1, 1);
% qd_desired(idx_hold_still:end, :) = zeros(size(qd_desired(idx_hold_still:end, :)));
% qdd_desired(idx_hold_still:end, :) = zeros(size(qdd_desired(idx_hold_still:end, :)));

torque = zeros(NUM_ITER, NUM_JOINTS);
for k = 1:NUM_ITER
    torque(k,:) = robot.rne(q(k,:), qd(k,:), qdd(k,:));
end

%% generate measurements
z = zeros(NUM_ITER, m);
for idx = 1:m
    z(:,idx) = torque(:,idx) + normrnd(0, measurement_sigma(idx), NUM_ITER, 1);
end

%% estimate parameters
disp('Start estimating!');

P_0 = zeros(1, n);
s_hat_1 = s_actual;

chosen_indices = [7,11,21]; % parameters being estimated
for idx = chosen_indices
    P_0(idx) = 1;
    s_hat_1(idx) = 1.5 * s_hat_1(idx);
end

% fine-tuning P_0...
P_0(7) = 0.01 * P_0(7);
P_0(11) = 5 * P_0(11);

P_0 = diag(P_0);

est_robot = robot_build_func(s_hat_1);

% ds = small difference in states used for numerical differentiation
% ds = max(s_hat_1 * 0.01, 0.001*ones(size(s_hat_1)));
ds = 1e-5 * ones(n, 1);

tic
[s_hat, H, residual, P_minus, P] = ...
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
YLIM_FACTOR = 2;

% Plot of H's condition number
H_cond = zeros(NUM_ITER, 1);
for k = 1:NUM_ITER
    H_cond(k) = cond(H(:,:,k));
end

figure;
plot(t, H_cond, 'DisplayName', 'Hs condition num', 'color', 'r');

title('Hs condition number vs. time');
xlabel('Time(s)');
ylabel('condition number');
legend('show');
saveas(gcf, strcat(folderName, '1-H condition number.jpg'));

OFFSET_DESCRIPTION_MAP = containers.Map({2, 3, 4}, {'x', 'y', 'z'});

for idx = 1:(n/10)
    base_idx = 10 * (idx-1);
    figure;
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

% Plot of residual
figure; hold on
for idx = 1:size(residual, 2)
    plot(t, residual(:,idx), 'DisplayName', sprintf('residual for torque %d', idx));
end

title('Residual vs. time');
xlabel('Time(s)');
ylabel('Residual torque(N*m)');

legend('show');
saveas(gcf, strcat(folderName, '5-residual.jpg'));

% Plot of P's norm
figure;
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

% Show final estimates
results = zeros(length(chosen_indices), 5);
for i = 1:length(chosen_indices)
    idx = chosen_indices(i);
    results(i,:) = [idx, P_0(idx, idx), s_actual(idx), s_hat(end,idx), ...
                    100*(s_hat(end,idx) - s_actual(idx))/s_actual(idx)];
end
dlmwrite(strcat(folderName, 'results.txt'), results, ' ');

%% More plots
% Plot of torques
figure; hold on
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
figure; hold on
for idx = 1:NUM_JOINTS
    plot(t, q(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('q vs. time');
xlabel('Time(s)');
ylabel('Joint angle(rad)');
legend('show');
saveas(gcf, strcat(folderName, '8-q.jpg'));

figure; hold on
for idx = 1:NUM_JOINTS
    plot(t, qd(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('qd vs. time');
xlabel('Time(s)');
ylabel('Joint velocity(rad/s)');
legend('show');
saveas(gcf, strcat(folderName, '9-qd.jpg'));

figure; hold on
for idx = 1:NUM_JOINTS
    plot(t, qdd(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('qdd vs. time');
xlabel('Time(s)');
ylabel('Joint angle(rad/s^2)');
legend('show');
saveas(gcf, strcat(folderName, '10-qdd.jpg'));

