function r = myRank(A, tol_multiplier)
    % Same as the Matlab rank() function but optionally applies the a
    % multiplier tol_multiplier to the tolerance value
    
    if ~exist('tol_multiplier','var')
        % optional parameter does not exist, so default it to something
        r = rank(A);
    else
        s = svd(A);
        tol = max(size(A))*eps(max(s)); % default tolerance used by Matlab
        tol = tol_multiplier * tol;
        r = sum(s > tol);
    end

end

