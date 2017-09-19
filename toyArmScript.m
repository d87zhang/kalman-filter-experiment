%% initialize
global dt t_f NUM_ITER n m;
dt = 0.01;
t_f = 5;
NUM_ITER = t_f / dt + 1;
t = linspace(0, t_f, NUM_ITER);
n = 6; % dimension of s
m = 2; % dimension of z
    
measurement_sigma = 8 * 1;
% measurement_sigma = 0;
assumed_measurement_sigma = measurement_sigma;

%% build robo
% dynamic parameters
link1_m = 3.7;
link1_COM_x = -0.8;
link1_inertia_about_z = link1_m * link1_COM_x^2;
link2_m = 8.2;
link2_COM_x = -1.1;
link2_inertia_about_z = link2_m * link2_COM_x^2;
% initial guesses
ig_link1_m = link1_m * 1.2;
ig_link1_COM_x = link1_COM_x * 1.2;
ig_link1_inertia_about_z = link1_inertia_about_z * 1.2;
ig_link2_m = link2_m * 1.2;
ig_link2_COM_x = link2_COM_x * 1.2;
ig_link2_inertia_about_z = link2_inertia_about_z * 1.2;

% TODO This fucks things up
% ig_link1_m = 3;
% ig_link1_COM_x = -0.3;
% ig_link1_inertia_about_z = ig_link1_m * ig_link1_COM_x^2;
% ig_link2_m = 8.7;
% ig_link2_COM_x = -0.7;
% ig_link2_inertia_about_z = ig_link2_m * ig_link2_COM_x^2;

s_actual = [link1_m, link1_COM_x, link1_inertia_about_z, ...
            link2_m, link2_COM_x, link2_inertia_about_z];
s_hat_1 = [ig_link1_m, ig_link1_COM_x, ig_link1_inertia_about_z, ...
           ig_link2_m, ig_link2_COM_x, ig_link2_inertia_about_z];
robot = buildPlaneMan(s_actual);

%% simulate everything
torque = zeros(NUM_ITER, m);
torque(:,1) = 100 * sin(2*pi/2.8 * t) + 10 * sin(2*pi/0.4 * t + pi);
torque(:,2) = 40 * sin(2*pi/1.7 * t + pi/2) + 7 * sin(2*pi/0.7 * t);
z = torque + normrnd(0, measurement_sigma, NUM_ITER, m);
[q, qd, qdd] = simulateRobo(robot, torque);
disp('Done simulating');

%% TODO testing
% robot.plot(q);
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
Q = repmat(0.1 * eye(n), 1, 1, NUM_ITER);
% TODO testing
% Q(:,:,1)
for k = 1:NUM_ITER
    Q(:,:,k) = decay_factors(k) * Q(:,:,k);
end

s_hat = estimateParams(z, assumed_measurement_sigma, Q, q, qd, qdd, s_hat_1);
disp('Done estimating');

%% Plot results!
folderName = 'C:\Users\Difei\Desktop\toyArm pics\currPlots\';
YLIM_FACTOR = 3;

hold on
% Plot of torque measurements
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
plot(t, s_hat(:,1), 'DisplayName', 'mass 1 est.', 'color', 'c');
plot(t, s_hat(:,4), 'DisplayName', 'mass 2 est.', 'color', 'magenta');

title('Mass est vs. time');
xlabel('Time(s)');
ylabel('mass(kg)');
max_abs = max(abs([link1_m, link2_m]));
ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);

saveas(gcf, strcat(folderName, '2-mass.jpg'));

% Plot of COM x pos estimates
figure; hold on
plot([0, t_f], s_actual(2) * ones(1, 2), 'DisplayName', 'actual COM x 1', 'color', 'b');
plot([0, t_f], s_actual(5) * ones(1, 2), 'DisplayName', 'actual COM x 2', 'color', 'r');
plot(t, s_hat(:,2), 'DisplayName', 'COM x 1 est.', 'color', 'c');
plot(t, s_hat(:,5), 'DisplayName', 'COM x 2 est.', 'color', 'magenta');

title('COM x est vs. time');
xlabel('Time(s)');
ylabel('COM x(m)');
max_abs = max(abs([link1_COM_x, link2_COM_x]));
ylim([max_abs * -YLIM_FACTOR, max_abs * YLIM_FACTOR]);

saveas(gcf, strcat(folderName, '3-COM x.jpg'));

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
  