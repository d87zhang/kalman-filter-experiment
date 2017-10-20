function setPlaneParams(robot, s)
    % change the whole set of params for the Plane Man
    
    robot.links(1).m = s(1);
    robot.links(2).m = s(2);
end