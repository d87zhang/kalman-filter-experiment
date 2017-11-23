function setPlaneParams(robot, s, s_spec)
    % change the whole set of params for the Plane Man

    assert(strcmp(s_spec, 'mass') | strcmp(s_spec, 'MoI'));
    
    if strcmp(s_spec, 'mass')
        robot.links(1).m = s(1);
        robot.links(2).m = s(2);
    else
        robot.links(1).I = [0 0 s(1)];
        robot.links(2).I = [0 0 s(2)];
    end
end