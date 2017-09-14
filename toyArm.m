dt = 0.01;
t_f = 4;
NUM_ITER = t_f / dt + 1;


%% Robo
L1 = 1;
L2 = 1.5;

links(1) = Link('d', 0, 'a', L1, 'alpha', pi);
links(2) = Link('d', 0, 'a', L2, 'alpha', 0);

GEAR_RATIO = 1; % arbitrary..
LINK_VFRICTION = 0; % TODO experiment with non-zero values? maybe estimate them as well?
LINK_MOTOR_INERTIA = 0;

% m   dynamic: link mass
% r   dynamic: link COG wrt link coordinate frame 3x1
% I   dynamic: link inertia matrix, symmetric 3x3, about link COG.
% B   dynamic: link viscous friction (motor referred)
% Tc  dynamic: link Coulomb friction
% G   actuator: gear ratio
% Jm  actuator: motor inertia (motor referred)

% dynamic parameters
link1_m = 3.7;
link1_COM_x = -0.2;
link1_inertia_about_z = link1_m * link1_COM_x^2;

link2_m = 8.2;
link2_COM_x = -1.1;
link2_inertia_about_z = link2_m * link2_COM_x^2;

links(1).m = link1_m;
links(1).r = [link1_COM_x 0 0]; % Must be a vector of 3 elements
links(1).I = [0 0 link1_inertia_about_z]; % Can be 3 or 6 element vector, or a 3x3 matrix
links(1).B = LINK_VFRICTION;
links(1).G = GEAR_RATIO;
links(1).Jm = LINK_MOTOR_INERTIA;

links(2).m = link2_m;
links(2).r = [link2_COM_x 0 0]; % Must be a vector of 3 elements
links(2).I = [0 0 link2_inertia_about_z]; % Can be 3 or 6 element vector, or a 3x3 matrix
links(2).B = LINK_VFRICTION;
links(2).G = GEAR_RATIO;
links(2).Jm = LINK_MOTOR_INERTIA;

robot = SerialLink(links, 'name', 'planeMan');

robot.gravity = [0 9.81 0]; % gravity goes in the -y direction
robot.qlim = [0 pi; 0 3/4*pi];
robot.plotopt = {'workspace' [-0.5,3, -3,3, -0.5,0.5]};

n = robot.n;

%% specifications
% s = [m1, com_x1, inertia_about_z1, m2, com_x2, inertia_about_z2, ...
%      q1, q2, qd1, qd2, ...
%      t1, t2]';


%% Simulation related stuff
% torque = repmat( sin(linspace(0, 2*pi, NUM_ITER))', 1, 2 );
torque = zeros(NUM_ITER, 2); % TODO testing

% initial state
q = zeros(NUM_ITER, n);
qd = zeros(NUM_ITER, n);

for iter = 2:NUM_ITER
    q_now = q(iter-1, :);
    qd_now = qd(iter-1, :);
    t_now = (iter - 1) * dt;
    [q_end, qd_end] = forwardDynamics(robot, t_now, q_now, qd_now, torque(iter-1, :), dt);
    q(iter, :) = q_end;
    qd(iter, :) = qd_end;
end

%% TODO testing
% q
% qd
robot.plot(q);

% initial guess
s_hat = [0.1, -0.7, 0.1 * 0.7^2, 0.1, -0.7, 0.1 * 0.7^2, ...
         pi/4, pi/4, 0, 0, 0, 0]'; 