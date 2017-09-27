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
