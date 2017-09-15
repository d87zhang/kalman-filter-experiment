global dt t_f NUM_ITER n m;

% dynamic parameters
link1_m = 3.7;
link1_COM_x = -0.2;
link1_inertia_about_z = link1_m * link1_COM_x^2;
link2_m = 8.2;
link2_COM_x = -1.1;
link2_inertia_about_z = 0.1;

s_actual = [link1_m, link1_COM_x, link1_inertia_about_z, ...
            link2_m, link2_COM_x, link2_inertia_about_z];

robot = buildPlaneMan(s_actual);

torque = zeros(NUM_ITER, m);
torque(:,end) = 10 * sin(linspace(0, 2*pi, NUM_ITER))';

[q, qd, qdd] = simulateRobo(robot, torque);

robot.plot(q);