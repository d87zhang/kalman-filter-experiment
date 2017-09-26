function robot = buildPlaneMan(s)
    % Returns a SerialLink robot built with the parameters from the given
    % state vector s
    
    L1 = 2;
    L2 = 3.5;

    links(1) = Link('d', 0, 'a', L1, 'alpha', pi);
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
    
    % dynamic parameters
    link1_m = s(1);
    link1_COM_x = s(2) / link1_m;
    link1_inertia_about_z = s(3);

    link2_m = s(4);
    link2_COM_x = s(5) / link2_m;
    link2_inertia_about_z = s(6);

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
    robot.plotopt = {'workspace' [-(L1 + L2 + 0.5),(L1 + L2 + 0.5), ...
                                  -(L1 + L2 + 0.5),(L1 + L2 + 0.5), -0.5,0.5]};
end
