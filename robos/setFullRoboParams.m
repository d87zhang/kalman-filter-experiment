function setFullRoboParams(robot, s, varargin)
    % change the whole set of params for a robot specified with 10n
    % parameters
    % Expects s to be composed of [m, m * center of mass, I] where center
    % of mass for link i is measured from frame i (see varargin for
    % exception)
    % varargin - one optional boolean variable indicating if the 2nd to 4th
    % state variables should be treated as simply center of mass instead of
    % mass * center of mass
    
    switch nargin - 2
        case 0
            using_mass_times_CoM = true;
        case 1
            using_mass_times_CoM = ~varargin{1};
        otherwise
            warning('setFullRoboParams() only accepts at most 1 varargin');
            assert(false);
    end
    
    assert(mod(length(s),10) == 0);
    for idx = 1:(length(s)/10)
        base_idx = 10 * (idx-1);
        
        robot.links(idx).m = s(base_idx + 1);
        
        if using_mass_times_CoM
            if robot.links(idx).m ~= 0
                robot.links(idx).r = s(base_idx + 2:base_idx + 4) / robot.links(idx).m;
            else
                robot.links(idx).r = zeros(size(robot.links(idx).r));
            end
        else
            robot.links(idx).r = s(base_idx + 2:base_idx + 4);
        end
        
        robot.links(idx).I = s(base_idx + 5:base_idx + 10);
    end
end