function corr_val = cleanAndGetCorr(A, B)
    % Filter out NaN and Inf values, then get the correlation value for the
    % given inputs

    assert(all(size(A) == size(B)));
    
    indices = ~isinf(A) & ~isnan(A) & ~isinf(B) & ~isnan(B); 
    corr_mat = corrcoef(A(indices), B(indices));
    corr_val = corr_mat(2,1);
end

