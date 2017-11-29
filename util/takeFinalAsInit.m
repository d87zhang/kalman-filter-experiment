% Script copying the final state of the filter to the initial state
% variables (i.e. setting up initial conditions using final state of
% estimator)

P_0 = P(:,:,end);
assert(all(size(s_hat_1) == size(s_hat(end, :)')));
s_hat_1 = s_hat(end, :)';

disp('initial conditions now matches the final filter state!');
