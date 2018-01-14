% Plot how correlation estimates for a parameter pair differ between trajectories
% requires some initial setup (running the first code section of
% main.m)

%% Step 1 - do estimation for the various trajectories
quintic_traj_rng_seeds = [666, 555, 444, 333, 222];
num_trajs = length(quintic_traj_rng_seeds);

corr_over_trajs = zeros([size(P_0), NUM_ITER, num_trajs]);
for traj_idx = 1:num_trajs
    % ========================================
    % (quintic splines) trajectory generation
    rng(quintic_traj_rng_seeds(traj_idx));
    t_sites = 0:2:t_f;
    NUM_SITES = length(t_sites);

    MAX_Y = 2.5;
    MAX_YD = 6;
    MAX_YDD = 20;
    q_spec = MAX_Y*rand(NUM_SITES, NUM_JOINTS) - MAX_Y/2;
    qd_spec = MAX_YD*rand(NUM_SITES, NUM_JOINTS) - MAX_YD/2;
    qdd_spec = MAX_YDD*rand(NUM_SITES, NUM_JOINTS) - MAX_YDD/2;

    [q, qd, qdd] = quinticSpline(q_spec, qd_spec, qdd_spec, t_sites, t);
    % ========================================
    
    % generate measurements
    torque = robot.rne([q, qd, qdd], robot.gravity, zeros(1,6));
    rng(65535);
    genMeas;
    
    % estimate parameters
    doTheEstimation;
    calcCorr;
    
    % record corr estimates
    corr_over_trajs(:,:,:,traj_idx) = corr;
end

disp('Done step 1!');

%% Step 2 - plot results
param_idx1 = 1;
param_idx2 = 2;

figure('units','normalized','outerposition',[0 0 1 1]); hold on;
for traj_idx = 1:num_trajs
    plot_target = corr_over_trajs(param_idx1,param_idx2,:,traj_idx);
    plot(t, reshape(plot_target, 1, numel(plot_target)), 'DisplayName', sprintf('traj - %d', traj_idx));
end

title_str = ['Correlation est. between ', getParamDescript(param_idx1, EST_CENTER_OF_MASS_ALONE), ' and ', ...
             getParamDescript(param_idx2, EST_CENTER_OF_MASS_ALONE), ' over diff trajs'];
title(title_str);
legend('show');
xlabel('Time(s)');

filename = regexprep(title_str, '*', 'x');
saveas(gcf, strcat(tempFolderName, [filename, '.jpg'])); % TODO remove these saveas() calls
