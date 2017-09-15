function torque = inverseDynamics(robot, q_now, qd_now, qdd_now)
    % TODO maybe try like Newton-Euler later?
    M = robot.inertia(q_now);
    C = robot.coriolis(q_now, qd_now);
    G = robot.gravload(q_now);
    torque = M*qdd_now' + C*qd_now' + G';
end