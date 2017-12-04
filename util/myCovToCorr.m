function corr_mat = myCovToCorr(cov)
    % my implementation of (the same functionality as) corrcov
    
    % method 1 - force symmetry and use corrcov
    
    % check if cov is roughly symmetric
    SYM_THRESHOLD = 1e-5;
    cov_sym = turnSym(cov);
    if any(any(abs(cov_sym - cov) > SYM_THRESHOLD))
        warning('[myCovToCorr] input matrix cov is not symmetric with tolerance of %f', SYM_THRESHOLD);
    end
    
    corr_mat = corrcov(cov_sym);
    
    % method 2
    % using formula from https://en.wikipedia.org/wiki/Covariance_matrix#Correlation_matrix
%     temp = diag( diag(cov).^(-1/2) );
%     corr_mat = temp * cov * temp;
end
