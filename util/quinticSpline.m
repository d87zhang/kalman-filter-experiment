function [q, qd, qdd] = quinticSpline(q_spec, qd_spec, qdd_spec, t_sites, t)
    % Returns a quintic spline in the form of q, qd and qdd each evaluated
    % at times given by vector t.
    % Boundary conditions for each piece-wise polynomial is given by
    % the q_spec, qd_spec and qdd_spec matrices where
    %   for any joint index j:
    %   let k be such that t(k) = t_sites(i), for some i
    %   q(k, j) = q_spec(i, j)
    %   qd(k, j) = qd_spec(i, j)
    %   qdd(k, j) = qdd_spec(i, j) , for all i = 1:length(t_sites)
    % Note: elements in t_sites must exist in t. 

    NUM_POINTS = length(t);
    NUM_SITES = length(t_sites);
    NUM_JOINTS = size(q_spec, 2);
    
    assert(all(size(q_spec) == [NUM_SITES, NUM_JOINTS]));
    assert(all(size(qd_spec) == size(q_spec)));
    assert(all(size(qdd_spec) == size(q_spec)));
    
    q = zeros(NUM_POINTS, NUM_JOINTS);
    qd = q;
    qdd = q;
    
    [~, t_indices] = ismember(t_sites, t);
    assert(all(t_indices) && issorted(t_indices, 'ascend'));
    
    for i = 1:NUM_SITES-1 % for each piece-wise polynomial
        t_start_idx = t_indices(i);
        t_end_idx = t_indices(i+1);
        [q_temp, qd_temp, qdd_temp] = ...
            quinticTraj(q_spec(i,:), qd_spec(i,:), qdd_spec(i,:), ...
                        q_spec(i+1,:), qd_spec(i+1,:), qdd_spec(i+1,:), ...
                        t(t_start_idx:t_end_idx));
        
        q(t_start_idx:t_end_idx,:) = q_temp;
        qd(t_start_idx:t_end_idx,:) = qd_temp;
        qdd(t_start_idx:t_end_idx,:) = qdd_temp;
    end
    
    %% correctness check
    for i = 1:length(t_sites)
        k = t_indices(i);
        assert(max(abs(q(k,:) - q_spec(i,:))) < 1e-8);
        assert(max(abs(qd(k,:) - qd_spec(i,:))) < 1e-8);
        assert(max(abs(qdd(k,:) - qdd_spec(i,:))) < 1e-8);
    end
end

