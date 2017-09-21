function [q, qd, qdd] = simulateRobo(robot, torque)
    % Simulate a constant robot
    global dt t_f NUM_ITER n m;

    q = zeros(NUM_ITER, m);
    qd = zeros(NUM_ITER, m);
    qdd = zeros(NUM_ITER, m);

    for iter = 2:NUM_ITER
        q_now = q(iter-1, :);
        qd_now = qd(iter-1, :);
        t_now = (iter - 1) * dt;
        [q_end, qd_end] = forwardDynamics(robot, t_now, q_now, qd_now, torque(iter-1, :), dt);
        q(iter, :) = q_end;
        qd(iter, :) = qd_end;
        
%         qdd(iter, :) = robot.accel(q_now, qd_now, torque(iter, :))';

%         qdd(iter, :) = (qd_end - qd_now)/dt;
    end
    
    for k = 2:NUM_ITER-1
        qdd(k,:) = (qd(k+1,:) - qd(k-1,:)) / (2*dt);
    end
    qdd(end,:) = (qd(end,:) - qd(end-1,:)) / dt;
end