%% initialize
global dt t_f NUM_ITER n m;
dt = 0.005;
t_f = 5;
NUM_ITER = t_f / dt + 1;
t = linspace(0, t_f, NUM_ITER)';
n = 3 * 10; % dimension of s
m = 3; % dimension of z
NUM_JOINTS = 6;

% measurement_sigma = 4 * 1e-1;
measurement_sigma = 0;
assumed_measurement_sigma = measurement_sigma;

%% build robo
% dynamic parameters
s_actual = zeros(n, 1);
s_actual(1:10) = [0, 0, 0, 0, 0   0   0.35    0   0   0];
s_actual(11:20) = [17.4, 17.4*0.068, 17.4*0.006, 17.4*-0.016, ...
                  .13   .524    .539    0     0   0];
s_actual(21:30) = [4.8, 0, 4.8*-0.070, 4.8*0.014, ...
                  .066    .0125   .066    0   0   0];

MASS_MULTIPLIER = 1; % for making a heavy version of PUMA for better sim
s_actual = MASS_MULTIPLIER * s_actual;
       
robot = buildPuma(s_actual);

%% generate torques and measurements
coef_file = matfile('coef.mat');
coef = coef_file.coef;
torque = genTorques(coef, t);
z = torque(:,1:m) + normrnd(0, measurement_sigma, NUM_ITER, m);

%% simulate robot
tic
[q, qd, qdd] = simulateRobo(robot, torque, dt);
toc

% simulate changing robot
% s = repmat(s_actual, NUM_ITER, 1);
% s(:,4) = linspace(s_actual(4), s_actual(4) + 3, NUM_ITER);
% tic
% [q, qd, qdd] = simulateChangingRobo(s, torque);
% toc

disp('Done simulating');

%% testing
% robot.plot(q);
calculated_torque = zeros(NUM_ITER, size(torque, 2));
for k = 1:NUM_ITER
    calculated_torque(k,:) = inverseDynamics(robot, q(k,:), qd(k,:), qdd(k,:));
end
norm(calculated_torque(:,1:m) - torque(:,1:m), 1)/numel(torque(:,1:m))

%% estimate parameters
s_hat_1 = 1.5 * s_actual; % TODO try something more fun

% Q anneals to zero (approaches zero) through exponential decay
decay_half_life = t_f/6;
alpha = -log(2) / decay_half_life;
decay_factors = exp(alpha * t);
% Q = repmat(0.01 * diag(s_hat_1), 1, 1, NUM_ITER);
Q = repmat(1 * eye(n), 1, 1, NUM_ITER);
for k = 1:NUM_ITER
    Q(:,:,k) = decay_factors(k) * Q(:,:,k);
end

tic
[s_hat, H, residual] = estimateParams(z, assumed_measurement_sigma, Q, q, qd, qdd, s_hat_1, @buildPuma);
toc
disp('Done estimating');

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