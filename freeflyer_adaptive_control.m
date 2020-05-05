% Compute adaptive control for the robot
function u = freeflyer_adaptive_control(x_des, x, xdd_des, ff, ff_model, Y)
    x = x(1:10);  % shave off parameter
    x_err = x - x_des;
   
    if ff.robot.n_q == 1
        u = -0.1*[eye(4) eye(4)]*x_err;
    elseif ff.robot.n_q == 2
        %% PD position controller
        Kp = 10*[3.0, 0, 0, 0, 0;
              0, 3.0, 0, 0, 0;
              0, 0, 0.1, 0, 0;
              0, 0, 0, 1.5, 0;
              0, 0, 0, 0, 1.5];
        Kd = 5.0*eye(5);
               
        u = -[Kp, Kd]*x_err;  % u = [F1 F2 tau0 tau1 tau2] INERTIAL
        
        %% Trajectory controller (feedback linearization)
        
%         % Use system model (best estimate of params)
%         C = ff_model.calc_CIM();
%         H = ff_model.calc_GIM();
%         
%         % Convert to Keenan convention: make 2D, rearrange state order
%         C = [C(4,:);   % rd1
%              C(5,:);   % rd2
%              C(3,:);   % w_z
%              C(7,:);   % qd1
%              C(8,:)];  % qd2
%         C = [C(:,4), C(:,5), C(:,3), C(:,7), C(:,8)];  % [F1 F2 tau0 tau1 tau2] 
%         
%         H = [H(4,:);   % rd1
%              H(5,:);   % rd2
%              H(3,:);   % w_z
%              H(7,:);   % qd1
%              H(8,:)];  % qd2
%         H = [H(:,4), H(:,5), H(:,3), H(:,7), H(:,8)];  % [F1 F2 tau0 tau1 tau2] 
%         
%         v = x(1:5);
%         
%         u = C*v + H*(-[Kp, Kd]*x_err + xdd_des);
        
        %% Adaptive controller (single unknown, m3)
%         lambda = 1.0;
%         gamma = 1.0;
%         Kds = 5.0*eye(5);
%         
%         xd_r = x_des(6:end) - 2*lambda*x_err(1:5);
%         xdd_r = xdd_des - 2*lambda*x_err(6:end);
%         s = x_err(6:end) + 2*lambda*x_err(1:5);
%         
%         Y_t = Y(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), xd_r(1), xd_r(2), xd_r(3), xd_r(4), xd_r(5), xdd_r(1), xdd_r(2), xdd_r(3), xdd_r(4), xdd_r(5), ff_model.a_hat);
%         Y_t = double(Y_t);
%         
% 
%         u = Y_t*ff_model.a_hat - Kds*s;
%         a_hatd = -gamma*Y_t'*s; % adaptation law
%         ff_model.a_hatd = a_hatd;
        
    elseif ff.robot.n_q == 3
        u = -0.005*[eye(6) eye(6)]*x_err; 
end