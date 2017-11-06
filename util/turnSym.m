function B = turnSym(A)
    % Returns a matrix which has the same lower triangular part as A and is
    % symmetric. A must be a square matrix
    B = tril(A) + tril(A,-1)';
end

