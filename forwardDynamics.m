function [q_end, qd_end] = forwardDynamics(robot, t_now, q_now, qd_now, torque, dt)
    %% calculate the forward dynamics dt time from t_now
    function xd = dynamics(~, x, r, tau)
      n = length(tau);
      q = x(1:n);
      qd = x(n+1:end);
      xd = [qd; r.accel(q', qd', tau)];
    end
    
    % Generate the forward dynamics for this time step
    [~, y] = ode45(@dynamics,[t_now, t_now + dt],[q_now qd_now]',[],robot,torque);

    % Convert the state back to a q and qd
    n = robot.n;
    q_end = y(end,1:n);
    qd_end = y(end,n+1:end);
end