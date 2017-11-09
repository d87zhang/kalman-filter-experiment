function corr_mat = myCovToCorr(cov)
    % my implementation of (the same functionality as) corrcov
    
    % method 1 - force symmetry and use corrcov
    corr_mat = corrcov(turnSym(cov));
    
    % method 2
    % using formula from https://en.wikipedia.org/wiki/Covariance_matrix#Correlation_matrix
%     temp = diag( diag(cov).^(-1/2) );
%     corr_mat = temp * cov * temp;
end
