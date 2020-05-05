% Create a reference trajectory for n-link manipulator to follow
function ref_traj = create_ref_traj()
    % [r1 r2 q0 q1 q2 r1d r2d q0d q1d q2d]'
    num_setpoints = 1001;
    tf = 10;  % approx final time
    dt = tf/num_setpoints;
    t_des_hist = linspace(0,tf,num_setpoints)';
    
    %% Circular
    x = pi*[0:.5:2]; 
    y = [0  0.0  -1.0  -2.0  -1.0  0  0; 
         1  0      1   0    -1.0  0  1];
    pp = spline(x,y);
    r = ppval(pp, linspace(0,2*pi,num_setpoints))';
    q0 = linspace(0,2*pi,num_setpoints)';
    
    samples1 = linspace(0,2*pi*3, num_setpoints);
    samples2 = linspace(0,2*pi*5, num_setpoints);
    q1 = (exp(-samples1./(2*pi*3).*.00001).*sin(samples1))';
    q2 = (exp(-samples2./(2*pi*5).*.00001).*sin(samples2))';
    
    x_des_hist = [r, q0, q1, q2];
    xd_des_hist = [diff(x_des_hist); zeros(1,5)]/dt;
    x_des_hist = [x_des_hist, xd_des_hist];
    
    xdd_des_hist = [diff(xd_des_hist); zeros(1,5)]/dt;  % accelerations
    
    

%     plot(r'(1,:),r'(2,:),'-b',y(1,2:5),y(2,2:5),'or')
%     axis equal

    %% Step Response
%     x_des_hist =  zeros(num_setpoints, 10);
%     x_des_hist(:,1:5) = repmat([0, 0, 0, 1.5, -1.5], num_setpoints, 1);
    


    ref_traj = [t_des_hist, x_des_hist xdd_des_hist];
end