%% initialize
global dt t_f NUM_ITER n m;
dt = 0.005;
t_f = 5;
NUM_ITER = t_f / dt + 1;
t = linspace(0, t_f, NUM_ITER)';
n = 3 * 10; % dimension of s
m = 6; % dimension of z
    
% measurement_sigma = 4 * 1;
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

s_hat_1 = 1.2 * s_actual; % TODO try something more fun
       
robot = buildPuma(s_actual);

%% generate torques and measurements
coef_file = matfile('coef.mat');
coef = coef_file.coef;
torque = genTorques(coef, t);
z = torque + normrnd(0, measurement_sigma, NUM_ITER, m);

%% simulate robot
tic
[q, qd, qdd] = simulateRobo(robot, torque);
toc

% simulate changing robot
% s = repmat(s_actual, NUM_ITER, 1);
% s(:,4) = linspace(s_actual(4), s_actual(4) + 3, NUM_ITER);
% tic
% [q, qd, qdd] = simulateChangingRobo(s, torque);
% toc

disp('Done simulating');

%% TODO testing
robot.plot(q);
% calculated_torque = zeros(NUM_ITER, m);
% for k = 1:NUM_ITER
%     calculated_torque(k,:) = inverseDynamics(robot, q(k,:), qd(k,:), qdd(k,:));
% end
% norm(calculated_torque - torque, 1)/numel(torque)

%% estimate parameters
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
[s_hat, H, residual] = estimateParams(z, assumed_measurement_sigma, Q, q, qd, qdd, s_hat_1, link1_COM_x, link2_COM_x);
toc
disp('Done estimating');

%% Plot results!
folderName = 'C:\Users\Difei\Desktop\toyArm pics\currPlots\';
YLIM_FACTOR = 3;

hold on
% Plot of torque measurements
plot([0, t_f], zeros(1, 2), 'DisplayName', 'zero line', 'color', 'black');
plot(t, torque(:,1), 'DisplayName', 'actual torque 1', 'color', 'b');
plot(t, torque(:,2), 'DisplayName', 'actual torque 2', 'color', 'r');
% plot(t, calculated_torque(:,1), 'DisplayName', 'calc torque 1', 'color', 'c');
% plot(t, calculated_torque(:,2), 'DisplayName', 'calc torque 2', 'color', 'magenta');
scatter(t, z(:,1), 'X', 'DisplayName', 'noisy torque 1 meas');
scatter(t, z(:,2), 'O', 'DisplayName', 'noisy torque 2 meas');

title('Torque vs. time');
xlabel('Time(s)');
ylabel('Torque(whatever torque is usually in)');

saveas(gcf, strcat(folderName, '1-torque.jpg'));

% Plot of mass estimates
figure; hold on
plot([0, t_f], s_actual(1) * ones(1, 2), 'DisplayName', 'actual mass 1', 'color', 'b');
plot([0, t_f], s_actual(4) * ones(1, 2), 'DisplayName', 'actual mass 2', 'color', 'r');
% plot(t, s(:,4), 'DisplayName', 'actual mass 2', 'color', 'r');
plot(t, s_hat(:,1), 'DisplayName', 'mass 1 est.', 'color', 'c');
plot(t, s_hat(:,4), 'DisplayName', 'mass 2 est.', 'color', 'magenta');

title('Mass est vs. time');
xlabel('Time(s)');
ylabel('mass(kg)');
max_abs = max(abs([link1_m, link2_m]));
ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);

saveas(gcf, strcat(folderName, '2-mass.jpg'));

% Plot of first moment of mass estimates
figure; hold on
plot([0, t_f], s_actual(2) * ones(1, 2), 'DisplayName', 'actual first moment 1', 'color', 'b');
plot([0, t_f], s_actual(5) * ones(1, 2), 'DisplayName', 'actual first moment 2', 'color', 'r');
plot(t, s_hat(:,2), 'DisplayName', 'first moment 1 est.', 'color', 'c');
plot(t, s_hat(:,5), 'DisplayName', 'first moment 2 est.', 'color', 'magenta');

title('first moment of mass est vs. time');
xlabel('Time(s)');
ylabel('first moment(kg*m)');
max_abs = max(abs([link1_m * link1_COM_x, link2_m * link2_COM_x]));
ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);

saveas(gcf, strcat(folderName, '3-first moment.jpg'));

% Plot of moment of inertia estimates
figure; hold on
plot([0, t_f], s_actual(3) * ones(1, 2), 'DisplayName', 'actual mass 1', 'color', 'b');
plot([0, t_f], s_actual(6) * ones(1, 2), 'DisplayName', 'actual mass 2', 'color', 'r');
plot(t, s_hat(:,3), 'DisplayName', 'mass 1 est.', 'color', 'c');
plot(t, s_hat(:,6), 'DisplayName', 'mass 2 est.', 'color', 'magenta');

title('Moment of inertia est vs. time');
xlabel('Time(s)');
ylabel('Moment of inertia(whatever torque is usually in)');
max_abs = max(abs([link1_inertia_about_z, link2_inertia_about_z]));
ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);

saveas(gcf, strcat(folderName, '4-moment of inertia.jpg'));

% Plot of residual
figure; hold on
plot(t, residual(:,1), 'DisplayName', 'residual for torque 1', 'color', 'b');
plot(t, residual(:,2), 'DisplayName', 'residual for torque 2', 'color', 'r');


title('Residual vs. time');
xlabel('Time(s)');
ylabel('Residual torque(whatever torque is usually in)');

saveas(gcf, strcat(folderName, '5-residual.jpg'));

% Plot of "magnitude" measures of H % TODO plot a measure of rankedness
% instead
% figure; hold on
% H_l1_norm = zeros(NUM_ITER, 1);clo
% for k = 1:NUM_ITER
%     H_l1_norm(k) = norm(H(:,:,k), 1);
% end
% H_max = zeros(NUM_ITER, 1);
% for k = 1:NUM_ITER
%     H_max(k) = max(max(H(:,:,k)));
% end
% plot(t, H_l1_norm / 9, 'DisplayName', 'H average magnitude', 'color', 'b');
% plot(t, H_max, 'DisplayName', 'H max magnitude', 'color', 'r');
% 
% title('H magnitude measures vs. time');
% xlabel('Time(s)');
% ylabel('unit for an element in H');

% saveas(gcf, strcat(folderName, '5-avg magnitude of H.jpg'));

% Plot of q, qd, qdd
% figure; hold on
% subplot(3, 1, 1);
% plot(t, q(:,1), 'DisplayName', 'joint angles 1', 'color', 'b');
% plot(t, q(:,2), 'DisplayName', 'joint angles 2', 'color', 'r');
% 
% title('Joint angles vs. time');
% xlabel('Time(s)');
% ylabel('Joint angle(rad)');
% 
% subplot(3, 1, 2);
% plot(t, qd(:,1), 'DisplayName', 'joint velocity 1', 'color', 'b');
% plot(t, qd(:,2), 'DisplayName', 'joint velocity 2', 'color', 'r');
% 
% title('Joint velocities vs. time');
% xlabel('Time(s)');
% ylabel('Joint velocity(rad/s)');
% 
% subplot(3, 1, 3);
% plot(t, qdd(:,1), 'DisplayName', 'joint acceleration 1', 'color', 'b');
% plot(t, qdd(:,2), 'DisplayName', 'joint acceleration 2', 'color', 'r');
% 
% title('Joint accelerations vs. time');
% xlabel('Time(s)');
% ylabel('Joint acceleration(rad/s^2)');