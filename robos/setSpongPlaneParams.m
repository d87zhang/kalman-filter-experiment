function setSpongPlaneParams(robot, s, L1, L2)
    % change the whole set of params for the 2-DoF robot described in the Spong book
    % p.2600
    % Expects s to be composed of [m, m * center of mass, I] where center
    % of mass for link i is measured from joint i
    
    s(2) = s(1) * (s(2)/s(1) - L1);
    s(12) = s(2) * (s(12)/s(11) - L2);
    
    setFullRoboParams(robot, s);
end