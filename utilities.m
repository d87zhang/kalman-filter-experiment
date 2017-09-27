%% Generate torque coefficients
m = 6;
num_sin = 5;
main_amplitudes = 1 * [80, 80, 60, 5, 5, 5]';
main_periods = [1.89, 2.6, 2.1, 3.1, 1.7, 2.3]';
secondary_amp_scale = 1/4;
secondary_period_scale = 1/3;

coef = zeros(m, num_sin * 3);
coef(:,1) = main_amplitudes;
coef(:,2) = main_periods;
for sin_idx = 0:num_sin-1
    if sin_idx ~= 0
        coef(:, sin_idx * 3 + 1) = 2 * secondary_amp_scale * main_amplitudes .* rand(m, 1);
        coef(:, sin_idx * 3 + 2) = 2 * secondary_period_scale * main_periods .* rand(m, 1);
    end
    % completely random phase offsets
    coef(:, sin_idx * 3 + 3) = pi * rand(m, 1);
end

save coef.mat coef;

%% see how far torque predictions differ..
s_hat_file = matfile('s_hat_vanilla.mat');
s_hat_test = s_hat_file.s_hat;

s_est = s_hat_test(end,:);
robot_est = buildPuma(s_est);

torque_est = zeros(NUM_ITER, size(torque, 2));
for k = 1:NUM_ITER
    torque_est(k,:) = inverseDynamics(robot_est, q(k,:), qd(k,:), qdd(k,:));
end
residual_est = torque_est - torque;

figure; hold on
for idx = 1:size(residual_est, 2)
    plot(t, residual_est(:,idx), 'DisplayName', sprintf('residual for torque %d', idx));
end

title('Residual vs. time');
xlabel('Time(s)');
ylabel('Residual torque(N*m)');

legend('show');
