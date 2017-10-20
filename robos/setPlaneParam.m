function setPlaneParam(robot, s_value, idx)
    % Change one param for the Plane Man
    
    assert(idx == 1 | idx == 2);
    robot.links(idx).m = s_value;
end