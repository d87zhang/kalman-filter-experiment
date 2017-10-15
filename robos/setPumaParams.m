function setPumaParams(robot, s)
    % change the whole set of params for the Puma robot
    for idx = 1:(length(s)/10)
        base_idx = 10 * (idx-1);
        robot.links(idx).m = s(base_idx + 1);
        robot.links(idx).r = s(base_idx + 2:base_idx + 4);
        robot.links(idx).I = s(base_idx + 5:base_idx + 10); % TODO think this is wrong
    end
end