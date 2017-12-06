function setSpongPlaneParam(robot, s_value, idx, L1, L2)
    % Change one param for the 2-DoF robot described in the Spong book
    % p.2600
    % Expects s to be composed of [m, m * center of mass, I] where center
    % of mass for link i is measured from joint i
    
    joint_idx = ceil(idx/10);
    param_idx = mod(idx, 10);
    
    switch joint_idx
        case 1
            L = L1;
        case 2
            L = L2;
        otherwise
            assert(false);
    end
    
    if param_idx == 1
        % changing mass: difficult case, don't reuse setFullRoboParam()
        m_new = s_value;
        m_old = robot.links(joint_idx).m;
        robot.links(joint_idx).m = m_new; % set m
        
        % in order to keep m * center of mass constant, have to change CoM
        % as well. In this case do so such that 
        % m_new * lc_new = m_old * lc_old
        lc_old = robot.links(joint_idx).r(1) + L;
        robot.links(joint_idx).r(1) = m_old/m_new * lc_old - L; % set r in x direction

        % set rest of r in same manner setFullRoboParam(), since
        % the difference in reference frame doesn't affect us here
        robot.links(joint_idx).r(2:3) = robot.links(joint_idx).r(2:3) ...
                                        * m_old / m_new;
    else
        if param_idx == 2
            % changing CoM in x direction, need to convert to other frame
            m = robot.links(joint_idx).m;
            s_value = m * (s_value/m - L);
        end

        setFullRoboParam(robot, s_value, idx);
    end
    
end