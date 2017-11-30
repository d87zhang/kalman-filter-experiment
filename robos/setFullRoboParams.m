function setFullRoboParams(robot, s)
    % change the whole set of params for a robot specified with 10n
    % parameters
    assert(mod(length(s),10) == 0);
    for idx = 1:(length(s)/10)
        base_idx = 10 * (idx-1);
        robot.links(idx).m = s(base_idx + 1);
        if robot.links(idx).m ~= 0
            robot.links(idx).r = s(base_idx + 2:base_idx + 4) / robot.links(idx).m;
        else
            robot.links(idx).r = zeros(size(robot.links(idx).r));
        end
        robot.links(idx).I = s(base_idx + 5:base_idx + 10);
    end
end