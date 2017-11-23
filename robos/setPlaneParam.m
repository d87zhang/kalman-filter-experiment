function setPlaneParam(robot, s_value, idx, s_spec)
    % Change one param for the Plane Man
    
    assert(strcmp(s_spec, 'mass') | strcmp(s_spec, 'MoI'));
    assert(idx == 1 | idx == 2);
    
    if strcmp(s_spec, 'mass')
        robot.links(idx).m = s_value;
    else
        robot.links(idx).I = [0 0 s_value];
    end
end