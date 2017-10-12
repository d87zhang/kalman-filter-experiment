%% do the tuning!
SAVE_PATH = 'saved_vars/tuning_R_data.mat';

% saving some initial things
save(SAVE_PATH, 'z', '-append');
save(SAVE_PATH, 'torque', '-append');
save(SAVE_PATH, 'q', '-append');
save(SAVE_PATH, 'qd', '-append');
save(SAVE_PATH, 'qdd', '-append');
save(SAVE_PATH, 'measurement_sigma', '-append');

save(SAVE_PATH, 's_hat_1', '-append');
save(SAVE_PATH, 'Q_1', '-append');


% set up test
NUM_TRIALS = 20;
R_factor = 0.5*(1.5.^(0:NUM_TRIALS-1));

save(SAVE_PATH, 'R_factor', '-append');
    
s_hat_results = zeros(NUM_ITER, n, NUM_TRIALS);
H_cond_results = zeros(NUM_ITER, NUM_TRIALS);
residual_results = zeros(NUM_ITER, m, NUM_TRIALS);

last_finished_trial = 0;

% run the test!
disp('Starting the experiment!');
for iter = 1:NUM_TRIALS
    tic
    assumed_measurement_sigma = measurement_sigma * R_factor(iter);
    [s_hat, H, residual, ~, ~] = estimateParams(z, assumed_measurement_sigma, Q, ...
                                                      q, qd, qdd, s_hat_1, ds, robot_build_func);
                                                  
    s_hat_results(:,:,iter) = s_hat;
    
    H_cond = zeros(NUM_ITER, 1);
    for k = 1:NUM_ITER
        H_cond(k) = cond(H(:,:,k));
    end
    H_cond_results(:,iter) = H_cond;
    
    residual_results(:,:,iter) = residual;
    toc
    
    fprintf('Done estimating iteration %d \\[T]/\n', iter);
    
    last_finished_trial = iter;
    
    disp('saving, please dun interrupt');
    save(SAVE_PATH, 's_hat_results', '-append');
    save(SAVE_PATH, 'H_cond_results', '-append');
    save(SAVE_PATH, 'residual_results', '-append');
    save(SAVE_PATH, 'last_finished_trial', '-append');
    disp('done saving!');
end

disp('all done!!!');
