function robot = buildPlaneManFull(s)
    % Returns a SerialLink robot built with the parameters from the given
    % state vector s (with 10n parameters)
    
    L1 = 2;
    L2 = 2;

    links(1) = Link('d', 0, 'a', L1, 'alpha', 0);
    links(2) = Link('d', 0, 'a', L2, 'alpha', 0);

    GEAR_RATIO = 1; % arbitrary..
    LINK_VFRICTION = 0;
    LINK_MOTOR_INERTIA = 0;

    % m   dynamic: link mass
    % r   dynamic: link COG wrt link coordinate frame 3x1
    % I   dynamic: link inertia matrix, symmetric 3x3, about link COG.
    % B   dynamic: link viscous friction (motor referred)
    % Tc  dynamic: link Coulomb friction
    % G   actuator: gear ratio
    % Jm  actuator: motor inertia (motor referred)
    
    links(1).B = LINK_VFRICTION;
    links(1).G = GEAR_RATIO;
    links(1).Jm = LINK_MOTOR_INERTIA;

    links(2).B = LINK_VFRICTION;
    links(2).G = GEAR_RATIO;
    links(2).Jm = LINK_MOTOR_INERTIA;

    robot = SerialLink(links, 'name', 'planeMan');

%     robot.gravity = [0 9.81 0]; % gravity goes in the -y direction (this
%     is prolly wrong)
    robot.gravity = [0 0 0]; % ignore gravity
    robot.qlim = [0 pi; 0 3/4*pi];
    robot.plotopt = {'workspace' [-(L1 + L2 + 0.5),(L1 + L2 + 0.5), ...
                                  -(L1 + L2 + 0.5),(L1 + L2 + 0.5), -0.5,0.5]};

    % set dynamic parameters
    setFullRoboParams(robot, s);
end
