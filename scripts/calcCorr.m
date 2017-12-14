% calculate correlation matrices by using and changing variables in the workspace
corr = zeros(size(P));
for k = 1:NUM_ITER
    corr(:,:,k) = myCovToCorr(P(:,:,k));
end
corr(isnan(corr)) = 0; % get rid of NaN's resulted from parameters with 0 variance
