function [ q, qd, qdd ] = linearTraj(a, b, t)
    % Returns a linear trajectory where each joint i has:
    %   q(:,i) = a(i)*t + b(i)
    %   qd(:,i) = a(i)
    %   qdd(:,i) = zeros
    % a and b must be vectors of equal size

    assert(length(a) == length(b));
    assert(size(t, 2) == 1); % t must be a column vector
    
    NUM_JOINTS = length(a);
    NUM_ITER = length(t);
    
    q = zeros(NUM_ITER, NUM_JOINTS);
    qd = q;
    qdd = q;
    
    for i = 1:NUM_JOINTS
        q(:,i) = a(i) * t + b(i);
        qd(:,i) = a(i) * ones(size(t));
        qdd(:,i) = zeros(size(t));
    end
end

