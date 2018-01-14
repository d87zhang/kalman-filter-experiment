% Script copying the final state of the filter to the initial state
% variables (i.e. setting up initial conditions using final state of
% estimator). To use, make sure you don't accidentally overwrite the newly
% setup initial conditions before estimating (i.e. skip first section of
% main.m?)

P_0 = P(:,:,end);
assert(all(size(s_hat_1) == size(s_hat(end, :)')));
s_hat_1 = s_hat(end, :)';

disp('initial conditions now matches the final filter state!');
