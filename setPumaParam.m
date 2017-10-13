function setPumaParam(robot, s_value, idx)
    % Change one param for the Puma robot
    joint_idx = ceil(idx/10);
    param_idx = mod(idx, 10);
    
    if param_idx == 1
        % it's a mass
        robot.links(joint_idx).m = s_value;
    elseif param_idx > 1 && param_idx < 5
        % it's a center of mass
        robot.links(joint_idx).r(param_idx - 1) = s_value;
    else
        % it's a moment of inertia
        switch param_idx
            case 5
                idx1 = 1;
                idx2 = 1;
            case 6
                idx1 = 2;
                idx2 = 2;
            case 7
                idx1 = 3;
                idx2 = 3;
            case 8
                idx1 = 2;
                idx2 = 1;
            case 9
                idx1 = 3;
                idx2 = 2;
            case 0
                idx1 = 3;
                idx2 = 1;
            otherwise
                assert(false);
        end
        
        I = robot.links(joint_idx).I;
        I(idx1, idx2) = s_value;
        I(idx2, idx1) = s_value;
        robot.links(joint_idx).I = I;
    end

end