%% initialize
global dt t_f NUM_ITER n m;
dt = 0.01;
t_f = 1;
NUM_ITER = t_f / dt + 1;
n = 6; % dimension of s
m = 2; % dimension of z
    
measurement_sigma = 0.05;
assumed_measurement_sigma = measurement_sigma;

%% build robo
% dynamic parameters
link1_m = 3.7;
link1_COM_x = -0.2;
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
torque_gain = 50;
torque = repmat( torque_gain * sin(linspace(0, 2*pi, NUM_ITER))', 1, m );
z = torque + normrnd(0, measurement_sigma, NUM_ITER, m);
[q, qd, qdd] = simulateRobo(robot, torque);

%% TODO testing
% robot.plot(q);

%% estimate parameters
s_hat = estimateParams(z, assumed_measurement_sigma, q, qd, qdd, s_hat_1);
       
%% Plot results!
hold on
% Plot of torque measurements
t = linspace(0, t_f, NUM_ITER);
plot(t, torque(:,1), 'DisplayName', 'actual torque 1');
plot(t, torque(:,2), 'DisplayName', 'actual torque 2');
scatter(t, z(:,1), 'X', 'DisplayName', 'noisy torque 1 meas');
scatter(t, z(:,2), 'O', 'DisplayName', 'noisy torque 2 meas');

title('Torque vs. time');
xlabel('Time(s)');
ylabel('Torque(whatever torque is usually in)');

% Plot of mass estimates
figure; hold on
plot([0, t_f], s_actual(1) * ones(1, 2), 'DisplayName', 'actual mass 1');
plot([0, t_f], s_actual(4) * ones(1, 2), 'DisplayName', 'actual mass 2');
plot(t, s_hat(:,1), 'DisplayName', 'mass 1 est.');
plot(t, s_hat(:,4), 'DisplayName', 'mass 2 est.');

title('Mass est vs. time');
xlabel('Time(s)');
ylabel('mass(kg)');
ylim([min(link1_m, link2_m) - 10, max(link1_m, link2_m) + 10]);

% Plot of COM x pos estimates
figure; hold on
plot([0, t_f], s_actual(2) * ones(1, 2), 'DisplayName', 'actual COM x 1');
plot([0, t_f], s_actual(5) * ones(1, 2), 'DisplayName', 'actual COM x 2');
plot(t, s_hat(:,2), 'DisplayName', 'COM x 1 est.');
plot(t, s_hat(:,5), 'DisplayName', 'COM x 2 est.');

title('COM x est vs. time');
xlabel('Time(s)');
ylabel('COM x(m)');
ylim([min(link1_COM_x, link2_COM_x) - 3, max(link1_COM_x, link2_COM_x) + 3]);

% Plot of moment of inertia estimates
figure; hold on
plot([0, t_f], s_actual(3) * ones(1, 2), 'DisplayName', 'actual mass 1');
plot([0, t_f], s_actual(6) * ones(1, 2), 'DisplayName', 'actual mass 2');
plot(t, s_hat(:,3), 'DisplayName', 'mass 1 est.');
plot(t, s_hat(:,6), 'DisplayName', 'mass 2 est.');

title('Moment of inertia est vs. time');
xlabel('Time(s)');
ylabel('Moment of inertia(whatever torque is usually in)');
ylim([min(link1_inertia_about_z, link2_inertia_about_z) - 10, ...
      max(link1_inertia_about_z, link2_inertia_about_z) + 10]);
