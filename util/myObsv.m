function Ob = myObsv(H, k)
    % Returns the observation matrix at iteration k assuming A_k = I (identity) and a
    % linear time-varying system with the output z_k = H_k * x_k
    % H is a m by n by NUM_ITER array where H(:,:,k) = H_k
    % Uses my own (don't know if correct) formula
    
    m = size(H, 1);
    n = size(H, 2);
    NUM_ITER = size(H, 3);
    
    num_stacks = min(n, NUM_ITER - k + 1);
    Ob = zeros(num_stacks * m, n);
    for i = 0:num_stacks-1
        Ob(i*m + 1:(i+1)*m, :) = H(:,:,k+i);
    end
end

