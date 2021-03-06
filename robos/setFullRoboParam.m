function setFullRoboParam(robot, s_value, idx, varargin)
    % Change one param for a robot specified with 10n parameters
    % Expects s to be composed of [m, m * center of mass, I] where center
    % of mass for link i is measured from frame i
    % varargin - one optional boolean variable indicating if the 2nd to 4th
    % state variables should be treated as simply center of mass instead of
    % mass * center of mass
    
    switch nargin - 3
        case 0
            using_mass_times_CoM = true;
        case 1
            using_mass_times_CoM = ~varargin{1};
        otherwise
            warning('setFullRoboParams() only accepts at most 1 varargin');
            assert(false);
    end
    
    joint_idx = ceil(idx/10);
    param_idx = mod(idx, 10);
    
    if param_idx == 1
        % it's a mass
        old_mass = robot.links(joint_idx).m;
        robot.links(joint_idx).m = s_value;
        
        if using_mass_times_CoM
            % also need to change the center of masses...
            if s_value ~= 0
                robot.links(joint_idx).r = robot.links(joint_idx).r ...
                    * old_mass / s_value;
            else
                robot.links(joint_idx).r = zeros(size(robot.links(joint_idx).r));
            end
        end
            
    elseif param_idx > 1 && param_idx < 5
        % it's center of mass related
        if using_mass_times_CoM
            % it's a mass * center of mass
            if robot.links(joint_idx).m ~= 0
                robot.links(joint_idx).r(param_idx - 1) = ...
                    s_value / robot.links(joint_idx).m;
            else
                robot.links(joint_idx).r(param_idx - 1) = 0;
            end
        else
            % it's just center of mass
            robot.links(joint_idx).r(param_idx - 1) = s_value;
        end
            
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