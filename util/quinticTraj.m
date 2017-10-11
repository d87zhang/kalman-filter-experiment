function [q, qd, qdd] = quinticTraj(q0, qd0, qdd0, qf, qdf, qddf, t)
    % Generates a quintic traj satisfying up to 2nd order boundary conditions
    % kinematic vectors are ... vectors, t is a column vector
    NUM_POINTS = length(t);
    t0 = t(1);
    tf = t(end);
    M = [1    t0    t0^2   t0^3      t0^4       t0^5;
         0    1     2*t0   3*t0^2    4*t0^3     5*t0^4;
         0    0     2      6*t0      12*t0^2    20*t0^3;
         1    tf    tf^2   tf^3      tf^4       tf^5;
         0    1     2*tf   3*tf^2    4*tf^3     5*tf^4;
         0    0     2      6*tf      12*tf^2    20*tf^3];

    n = length(q0);
    q = zeros(NUM_POINTS, n);
    qd = zeros(NUM_POINTS, n);
    qdd = zeros(NUM_POINTS, n);
    onez = ones(NUM_POINTS, 1);

    % for each joint, find a fitting quintic function and generate
    % the trajectory
    for jointIdx = 1:n
        b = [q0(jointIdx); ...
             qd0(jointIdx); ...
             qdd0(jointIdx); ...
             qf(jointIdx); ...
             qdf(jointIdx); ...
             qddf(jointIdx)];
        a = M\b;
        q(:, jointIdx) = a(1).*onez + a(2).*t +a(3).*(t.^2) + a(4).*(t.^3) ...
                         + a(5).*(t.^4) + a(6).*(t.^5);
        qd(:, jointIdx) = a(2).*onez +2*a(3).*t +3*a(4).*(t.^2) ...
                          + 4*a(5).*(t.^3) + 5*a(6).*(t.^4);
        qdd(:, jointIdx) = 2*a(3).*onez + 6*a(4).*t + 12*a(5).*(t.^2) ...
                           + 20*a(6).*(t.^3);
    end


end
