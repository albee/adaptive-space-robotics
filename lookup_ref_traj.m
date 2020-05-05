% Provide the time-parameterized reference trajectory
function [x_des, xdd_des] = lookup_ref_traj(t, ff, ref_traj)
    n = 2*(3 + ff.robot.n_q);

    % sudden jump
    if ff.robot.n_q == 1
        x_des = [0; 0; 0; 1.5; 0; 0; 0; 0];
    elseif ff.robot.n_q == 2
%         x_des = [0; 0; 0; 1.5; -1.5; 0; 0; 0; 0; 0]; 

        % get closest reference point
        t_des_hist = ref_traj(:,1);
        x_des_hist = ref_traj(:, 2:n+1);
        xdd_des_hist = ref_traj(:, n+2:end);
        
        cut = t_des_hist(t_des_hist <= t);
        cut_idx = length(cut);
        
        x_des = x_des_hist(cut_idx, :)';
        xdd_des = xdd_des_hist(cut_idx, :)';
    elseif ff.robot.n_q == 3
        x_des = [0; 0; 0; 1.5; -1.5; 1.5; 0; 0; 0; 0; 0; 0]; 
end