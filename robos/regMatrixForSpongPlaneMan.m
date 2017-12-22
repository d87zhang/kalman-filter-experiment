function Y = regMatrixForSpongPlaneMan(q, qd, qdd)
    % Returns the regressor matrix Y(q, qd, qdd) over potentially multiple
    % timesteps for the 2-DoF revolute arm as specified in the Spong textbook
    % q, qd, qdd are expected to be NUM_ITER x NUM_JOINTS
    
    NUM_ITER = size(q, 1);
    NUM_JOINTS = size(q, 2);
    
    assert(NUM_JOINTS == 2);
    assert(all(size(q) == size(qd)) && all(size(q) == size(qdd)));
    
    Y = zeros(NUM_JOINTS, 5, NUM_ITER);
    for k = 1:NUM_ITER
        Y(:,:,k) = regMatrix(q(k,:), qd(k,:), qdd(k,:));
    end
    
    function Y = regMatrix(q, qd, qdd)
        % Returns the regressor matrix Y(q, qd, qdd) at a particular time 
        % for the 2-DoF revolute arm as specified in the Spong textbook 
        % (equations come from p.271).
        % q, qd, qdd are expected to be vectors of size NUM_JOINTS
        g = 9.81;
        
        Y = zeros(NUM_JOINTS, 5);
        Y(1,1) = qdd(1);
        Y(1,2) = cos(q(2)) * (2*qdd(1) + qdd(2)) - sin(q(2)) * (qd(1)^2 + 2*qd(1)*qd(2));
        Y(1,3) = qdd(2);
        Y(1,4) = g * cos(q(1));
        Y(1,5) = g * cos(q(1) + q(2));
        Y(2,2) = cos(q(2)) * qdd(1) + sin(q(2)) * qd(1)^2;
        Y(2,3) = qdd(1) + qdd(2);
        Y(2,5) = g * cos(q(1) + q(2));
    end
end

