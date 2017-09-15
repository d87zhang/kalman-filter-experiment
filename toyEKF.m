%% port of example code from: http://scipy-cookbook.readthedocs.io/items/KalmanFiltering.html?highlight=kalman%20filter

n_iter = 50;
x = -0.37727; % truth value (typo in example at top of p. 13 calls this z)
z = normrnd(x, 0.1, n_iter, 1); % observations (normal about x, sigma=0.1)

Q = 1e-5; % process variance

% allocate space for arrays
xhat = zeros(n_iter, 1);      % a posteri estimate of x
P = zeros(n_iter, 1);         % a posteri error estimate
xhatminus = zeros(n_iter, 1); % a priori estimate of x
Pminus = zeros(n_iter, 1);    % a priori error estimate
K = zeros(n_iter, 1);         % gain or blending factor

R = 0.1^2; % estimate of measurement variance, change to see effect
% R = 1;

% intial guesses
xhat(1) = 0.0;
P(1) = 1.0; % apparently, this doesn't really matter so long as it's non-zero

for k = 2:n_iter
    % time update
    xhatminus(k) = xhat(k-1);
    Pminus(k) = P(k-1)+Q;

    % measurement update
    K(k) = Pminus(k)/( Pminus(k)+R );
    xhat(k) = xhatminus(k)+K(k)*(z(k)-xhatminus(k));
    P(k) = (1-K(k))*Pminus(k);
end


iters = 1:50;

%% Graphing
hold on
z_plot = scatter(iters, z, 'X', 'DisplayName', 'noisy measurements');
xhat_plot = plot(iters, xhat, 'DisplayName', 'a posteri estimate');
x_plot = plot(iters, repmat(x, n_iter, 1), 'DisplayName', 'truth value');

% legend([z_plot, xhat_plot, x_plot], ['noisy measurements', 'a posteri estimate', 'truth value']);
title('Estimate vs. iteration step');
xlabel('Iteration');
ylabel('Voltage');


figure;

valid_iter = iters(2:end); % because Pminus not valid at step 0

plot(valid_iter, Pminus(valid_iter), 'DisplayName', 'a priori error estimate');
title('Estimated a priori error vs. iteration step');

xlabel('Iteration');
ylabel('$(Voltage)^2$');
% ylim([0 .01]);
hold off

