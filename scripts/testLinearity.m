% Pre-requisites: trajectory is prepared to sample from (q, qd, qdd,
% NUM_ITER are populated); also other things from the regular estimation
% setup (like robot, etc.)...

s_scales = [1/4, 1/2, 1, 2, 4, 10];
s_baseline = s_actual; % doesn't have to be actual values

torques_by_scale = containers.Map('KeyType', 'double', 'ValueType', 'any');

for scale = s_scales
    robot_set_params_func(robot, scale * s_baseline);
    torques_by_scale(scale) = robot.rne([q, qd, qdd], robot.gravity, zeros(1,6));
end

%% Analyzing results
reshaped_torques_by_scale_idx = zeros(length(s_scales), ...
    size(torques_by_scale(1), 1) * size(torques_by_scale(1), 2));
for i = 1:length(s_scales)
    scale = s_scales(i);
    reshaped_torques_by_scale_idx(i,:) = reshape(torques_by_scale(scale), ...
        1, size(reshaped_torques_by_scale_idx, 2));
    assert(all(~isinf(reshaped_torques_by_scale_idx(i,:)) ...
        & ~isnan(reshaped_torques_by_scale_idx(i,:))));
end

corr_coefs = diag(ones(1, length(s_scales)));
for i = 1:length(s_scales)-1
    for j = i+1:length(s_scales)
        corr_coefs(i,j) = cleanAndGetCorr(...
            reshaped_torques_by_scale_idx(i,:), ...
            reshaped_torques_by_scale_idx(j,:));
    end
end

disp(corr_coefs);

%% looking at specific things
torques_1_scale = 1;
torques_2_scale = 2;
torques_1 = torques_by_scale(torques_1_scale);
torques_2 = torques_by_scale(torques_2_scale);

torque_idx = 1;

for torque_idx = 1:NUM_JOINTS
    figure('units','normalized','outerposition',[0 0 1 1]); hold on;
    plot(t, torques_1(:,torque_idx), 'DisplayName', sprintf('scale = %d', torques_1_scale));
    plot(t, torques_2(:,torque_idx), 'DisplayName', sprintf('scale = %d', torques_2_scale));
    plot(t, torques_2(:,torque_idx) - torques_1(:,torque_idx), 'DisplayName', 'the diff');
    legend('show');
    xlabel('Time(s)');
    ylabel('Torque(N*m)');
    title_text = sprintf('Torque at link %d vs time under different state scalings', torque_idx);
    title(title_text);
    
    saveas(gcf, strcat(folderName, [title_text '.jpg']));
end