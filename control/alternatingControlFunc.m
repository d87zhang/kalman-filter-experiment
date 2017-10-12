function torque = alternatingControlFunc(t_now, q_desired, q_now, qd_desired, qd_now, ...
                                         Kp, Kd, NUM_IGNORED_JOINTS, ...
                                         robot)
    % A controller that can alternate between two control strategies
    ALT_THRESHOLD = 9999;
    
    torque = zeros(size(q_now));
    if t_now < ALT_THRESHOLD
        torque = pdControlFunc(t_now, q_desired, q_now, qd_desired, qd_now, ...
                               Kp, Kd, NUM_IGNORED_JOINTS);
    end
    
    % always compensate for gravity
    torque = torque + robot.gravload(q_now);
end

