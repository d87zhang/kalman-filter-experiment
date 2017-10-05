%% initialize
global t_f NUM_ITER n m;
dt = 0.001;
t_f = 5;
NUM_ITER = t_f / dt + 1;
t = linspace(0, t_f, NUM_ITER)';
n = 3 * 10; % dimension of s
% n = 3;
m = 3; % dimension of z
NUM_JOINTS = 6;

% measurement_sigma = 10 * 1e0;
measurement_sigma = 0;
% assumed_measurement_sigma = measurement_sigma;
assumed_measurement_sigma = measurement_sigma + 50;

% build robo
robot_build_func = @buildPuma;
% dynamic parameters
s_actual = zeros(n, 1);
s_actual(1:10) = [0, 0, 0, 0, 0   0   0.35    0   0   0];
s_actual(11:20) = [17.4, 17.4*0.068, 17.4*0.006, 17.4*-0.016, ...
                  .13   .524    .539    0     0   0];
s_actual(21:30) = [4.8, 0, 4.8*-0.070, 4.8*0.014, ...
                  .066    .0125   .066    0   0   0];
% here s contains 3 elements for each link: m, first moment about x, moment
% of inertia about z (zz)
% s_actual = [0, 0, 0.35, ...
%             17.4, 17.4*0.068, .539, ...
%             4.8, 4.8*-0.070, .066];
% s_actual = [0.35, 17.4, 4.8];

MASS_MULTIPLIER = 1; % for making a heavy version of PUMA for better sim
s_actual = MASS_MULTIPLIER * s_actual;        

% build robo (but it fake)
% robot_build_func = @(s)(FakeRobo(s, NUM_JOINTS));
% s_actual = [2.4, 6, 3.7, 7.3, 4];

robot = robot_build_func(s_actual);

% this is so important :)
[sf1_y, sf1_Fs] = audioread('soundeffects\militarycreation.wav');
[sf2_y, sf2_Fs] = audioread('soundeffects\villagercreation.wav');
[sf3_y, sf3_Fs] = audioread('soundeffects\academy.wav');
[sf4_y, sf4_Fs] = audioread('soundeffects\barracks.wav');

%% trajectory gen
coef_file = matfile('coef.mat');
coef = coef_file.ff_coef;
t_offsets = [0.4, -0.8, 0.7, 0 0 0];
[q_desired, qd_desired, qdd_desired] = genFFS(coef, t, t_offsets);

%% simulate robot
disp('Start simulating!');
Kp = [150 240 120 0 0 0]'; % proportional gain
Kd = [33 41 16 0 0 0]'; % differential gain
controlFunc = @(t_now, q_desired, q_now, qd_desired, qd_now) ...
               (pdControlFunc(t_now, q_desired, q_now, qd_desired, qd_now, ...
                              Kp, Kd, 3));

% for fake robot
% simple_coef = coef_file.simple_ff_coef;
% control_out = genFFS(simple_coef, t, [0, 1, 2]);
% controlFunc = @(t_now, q_desired, q_now, qd_desired, qd_now) ...
%                (control_out(round(t_now/dt) + 1,:));

tic
[q, qd, qdd, torque] = simulateRobo(robot, controlFunc, q_desired, qd_desired, t);
toc

% simulate changing robot
% s = repmat(s_actual, NUM_ITER, 1);
% s(:,4) = linspace(s_actual(4), s_actual(4) + 3, NUM_ITER);
% tic
% [q, qd, qdd] = simulateChangingRobo(s, torque);
% toc

disp('Done simulating \[T]/');
if rand() > 0.5
    sound(sf1_y, sf1_Fs);
else
    sound(sf2_y, sf2_Fs);
end

%% generate measurements
z = torque(:,1:m) + normrnd(0, measurement_sigma, NUM_ITER, m);

%% testing
calculated_torque = zeros(NUM_ITER, size(torque, 2));
for k = 1:NUM_ITER
    calculated_torque(k,:) = inverseDynamics(robot, q(k,:), qd(k,:), qdd(k,:));
end
norm(calculated_torque(:,1:m) - torque(:,1:m), 1)

% % plot the f*cker
% figure; hold on
% for idx = 1:m
%     plot(t, torque(:, idx), 'DisplayName', sprintf('true torque - %d', idx));
%     plot(t, calculated_torque(:, idx), 'DisplayName', sprintf('calc torque - %d', idx));
% end
% legend('show');
% 
% figure; hold on
% for idx = 1:m
%     plot(t, calculated_torque(:, idx), 'DisplayName', sprintf('calc torque - %d', idx));
% end
% legend('show');
% title('Just the calculated torques');

%% estimate parameters
disp('Start estimating!');
s_hat_1 = 1.5 * s_actual; % TODO try something more fun

% method 1: scale Q according to initial guesses
% Q_1 = 5 * diag(min(s_hat_1.^2, 0.1 * ones(size(s_hat_1))));

% method 2: set Q to identity, but scale parts from experience...
Q_1 = 1 * eye(n);
% boost Q for mass parameters
for link_idx = 1:n/10
    idx = 10*(link_idx-1) + 1;
    Q_1(idx, idx) = 1000;
end
% lower moment of inertia's Q parameters
Q_1(25:30,:) = 0.01 * Q_1(25:30,:);

Q = repmat(Q_1, 1, 1, NUM_ITER);

% % Q anneals to zero (approaches zero) through exponential decay
decay_half_life = t_f/6;
alpha = -log(2) / decay_half_life;
decay_factors = exp(alpha * t);
for k = 1:NUM_ITER
    Q(:,:,k) = decay_factors(k) * Q(:,:,k);
end

% ds = small difference in states used for numerical differentiation
ds = max(s_hat_1 * 0.01, 0.001*ones(size(s_hat_1)));

tic
[s_hat, H, residual, P_minus, P] = estimateParams(z, assumed_measurement_sigma, Q, ...
                                                  q, qd, qdd, s_hat_1, ds, robot_build_func);
toc
disp('Done estimating \[T]/');
if rand() > 0.5
    sound(sf3_y, sf1_Fs);
else
    sound(sf4_y, sf2_Fs);
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
title(sprintf('Hs condition number vs. time (meas sigma = %0.2e)', measurement_sigma));
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
    
    title(sprintf('link %d - first moment of mass est vs. time', idx));
    xlabel('Time(s)');
    ylabel('first moment(kg*m)');
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

% for idx = 1:3
%     figure
% %     subplot(3, 1, 1); hold on
%     hold on;
%     plot([0, t_f], s_actual(1*(idx-1) + 1) * ones(1, 2), 'color', 'b');
%     plot(t, s_hat(:,1*(idx-1) + 1), 'color', 'r');
%     title(sprintf('link %d - something vs. time', idx));
%     xlabel('Time(s)');
%     ylabel('something(??)');
%     max_abs = max([abs(s_actual(1*(idx-1) + 1)), 0.1]);
%     ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);
%     
%     saveas(gcf, strcat(folderName, sprintf('%d-estimate joint %d.jpg', idx+1, idx)));
%     
% %     subplot(3, 1, 2); hold on
% %     plot([0, t_f], s_actual(3*(idx-1) + 2) * ones(1, 2), 'color', 'b');
% %     plot(t, s_hat(:,3*(idx-1) + 2), 'color', 'r');
% %     title(sprintf('link %d - first moment about x vs. time', idx));
% %     xlabel('Time(s)');
% %     ylabel('first moment(kg*m)');
% %     max_abs = max([abs(s_actual(3*(idx-1) + 2)), 0.1]);
% %     ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);
% %     
% %     subplot(3, 1, 3); hold on
% %     plot([0, t_f], s_actual(3*(idx-1) + 3) * ones(1, 2), 'color', 'b');
% %     plot(t, s_hat(:,3*(idx-1) + 3), 'color', 'r');
% %     title(sprintf('link %d - moment of inertia vs. time', idx));
% %     xlabel('Time(s)');
% %     ylabel('moment of inertia(kg*m^2)');
% %     max_abs = max([abs(s_actual(3*(idx-1) + 3)), 0.1]);
% %     ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);
% end

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

%% More plots
% Plot of torques
figure; hold on
for idx = 1:NUM_JOINTS
    plot(t, torque(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('Torque vs. time');
xlabel('Time(s)');
ylabel('Torque(N*m)');
legend('show');
saveas(gcf, strcat(folderName, '6-torques.jpg'));

%Plot of q's
figure; hold on
for idx = 1:NUM_JOINTS
    plot(t, q(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('q vs. time');
xlabel('Time(s)');
ylabel('Joint angle(rad)');
legend('show');
saveas(gcf, strcat(folderName, '7-q.jpg'));

figure; hold on
for idx = 1:NUM_JOINTS
    plot(t, qd(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('qd vs. time');
xlabel('Time(s)');
ylabel('Joint velocity(rad/s)');
legend('show');
saveas(gcf, strcat(folderName, '8-qd.jpg'));

figure; hold on
for idx = 1:NUM_JOINTS
    plot(t, qdd(:,idx), 'DisplayName', sprintf('joint %d', idx));
end
title('qdd vs. time');
xlabel('Time(s)');
ylabel('Joint angle(rad/s^2)');
legend('show');
saveas(gcf, strcat(folderName, '9-qdd.jpg'));

