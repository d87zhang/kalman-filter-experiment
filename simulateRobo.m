function [q, qd, qdd, torque] = simulateRobo(robot, controlFunc, q_desired, qd_desired, t)
    % Simulate a constant robot
    % controlFunc - function of the form:
    %     torque = controlFunc(t_now, q_desired, q_now, qd_desired, qd_now)
    %     where torque, q_desired, q_now, qd_desired, qd_now are row 
    %     vectors; torque has length NUM_JOINTS
    % q_desired - NUM_ITER x m matrix of joint angles representing a
    %             desired trajectory
    % qd_desired - similar to q_desired (but representing velocity)
    
    NUM_ITER = size(t, 1);
    NUM_JOINTS = robot.n;
    
    % these will be populated iteratively
    q = zeros(NUM_ITER, NUM_JOINTS);
    qd = q;
    qdd = q;
    torque = zeros(NUM_ITER, NUM_JOINTS);

    assert(t(1) == 0);
    
    for iter = 2:NUM_ITER
        % simulate kinematics at iter
        q_now = q(iter-1, :);
        qd_now = qd(iter-1, :);
        t_now = t(iter-1);
        dt = t(iter) - t_now;
        
        % compute control output (torque)
        torque(iter-1,:) = controlFunc(t_now, ...
                                       q_desired(iter-1,:), q_now, ...
                                       qd_desired(iter-1,:), qd_now);
        
        [q_end, qd_end] = forwardDynamics(robot, t_now, q_now, qd_now, torque(iter-1,:), dt);
        q(iter, :) = q_end;
        qd(iter, :) = qd_end;
        
%         qdd(iter, :) = robot.accel(q_now, qd_now, torque(iter, :))';
    end
    
    % calculate torque at the end just for kicks
    torque(end,:) = controlFunc(t(end), q_desired(end,:), q(end,:), ...
                                qd_desired(end,:), qd(end,:));
    
    % calculate qdd...
    for k = 2:NUM_ITER-1
        qdd(k,:) = (qd(k+1,:) - qd(k-1,:)) / (2*dt);
    end
    qdd(end,:) = (qd(end,:) - qd(end-1,:)) / dt;
end