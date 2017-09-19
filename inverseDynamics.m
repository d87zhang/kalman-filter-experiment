function torque = inverseDynamics(robot, q_now, qd_now, qdd_now)
%     M = robot.inertia(q_now);
%     C = robot.coriolis(q_now, qd_now);
%     G = robot.gravload(q_now);
%     torque = M*qdd_now' + C*qd_now' + G';

    % RNE seems to be faster
    torque = robot.rne(q_now, qd_now, qdd_now)';
end