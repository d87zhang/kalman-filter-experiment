% estimates parameters by using and changing variables in the workspace
disp('Start estimating!');

% assumes initial conditions: s_hat_1 and P_0 are populated
est_robot = robot_build_func(s_hat_1);

% ds = small difference in states used for numerical differentiation
% ds = max(s_hat_1 * 0.01, 0.001*ones(size(s_hat_1)));
ds = 1e-5 * ones(n, 1);

tic
[s_hat, H, residual, K, P] = ...
    estimateParams(z, assumed_measurement_sigma, P_0, Q, ...
                   q, qd, qdd, s_hat_1, ds, ...
                   est_robot, robot_set_param_func, robot_set_params_func);
toc
disp('Done estimating \[T]/');
if rand() > 0.5
    sound(sf1_y, sf1_Fs);
else
    sound(sf2_y, sf2_Fs);
end