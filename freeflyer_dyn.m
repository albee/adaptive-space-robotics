function dxdt = freeflyer_dyn(t, x, ff, ff_model, ref_traj, Y)
    % 1-link free-flying manipulator dynamics (no gravity)
    %
    % Inputs:
    % t         (1x1) time vector
    % x         (8x1) state vector [r1 r2 q0 q1 r1d r2d q0d q1d]'
    %                              [r1 r2 q0 q1 q2 r1d r2d q0d q1d q2d]'
    %
    % Output:
    % dxdt      (8x1) derivative of state vector [r1d r2d q0d q1d r1dd r2dd q0dd q1dd]'
 
    % Unpack state vec 
    if ff.robot.n_q == 1
        r1 = x(1);
        r2 = x(2);
        q0 = x(3);
        q1 = x(4);
        r1d = x(5);
        r2d = x(6);
        q0d = x(7);
        q1d = x(8);
                
        % Set ff params
        ff.r0 = [r1; r2; 0];
        ff.R0 = eul2rotm([q0, 0, 0]);
        ff.qm = [q1];
        ff.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff.um = [q1d];
        
        ff_model.r0 = [r1; r2; 0];
        ff_model.R0 = eul2rotm([q0, 0, 0]);
        ff_model.qm = [q1; q2];
        ff_model.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff_model.um = [q1d; q2d];
    elseif ff.robot.n_q == 2
        r1 = x(1);
        r2 = x(2);
        q0 = x(3);
        q1 = x(4);
        q2 = x(5);
        r1d = x(6);
        r2d = x(7);
        q0d = x(8);
        q1d = x(9);
        q2d = x(10);
        
        % Set ff params
        ff.r0 = [r1; r2; 0];
        ff.R0 = eul2rotm([q0, 0, 0]);
        ff.qm = [q1; q2];
        ff.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff.um = [q1d; q2d];
        
        ff_model.r0 = [r1; r2; 0];
        ff_model.R0 = eul2rotm([q0, 0, 0]);
        ff_model.qm = [q1; q2];
        ff_model.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff_model.um = [q1d; q2d];
        
    elseif ff.robot.n_q == 3
        r1 = x(1);
        r2 = x(2);
        q0 = x(3);
        q1 = x(4);
        q2 = x(5);
        q3 = x(6);
        r1d = x(7);
        r2d = x(8);
        q0d = x(9);
        q1d = x(10);
        q2d = x(11);
        q3d = x(12);
        
        % Set ff params
        ff.r0 = [r1; r2; 0];
        ff.R0 = eul2rotm([q0, 0, 0]);
        ff.qm = [q1; q2; q3];
        ff.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff.um = [q1d; q2d; q3d];
        
        ff_model.r0 = [r1; r2; 0];
        ff_model.R0 = eul2rotm([q0, 0, 0]);
        ff_model.qm = [q1; q2; q3];
        ff_model.u0 = [0; 0; q0d; r1d; r2d; 0];
        ff_model.um = [q1d; q2d; q3d];
    end

    % Constants

    % Dynamics params
    
    %% Desired traj.
    [x_des, xdd_des] = lookup_ref_traj(t, ff, ref_traj);

    %% Control
    control = freeflyer_adaptive_control(x_des, x, xdd_des, ff, ff_model, Y);  % u = [F1 F2 tau0 tau1 tau2] INERTIAL

    %% Dynamics, using the SPART interface for forward dynamics
    
    % Set wrenches (F/T in INERTIAL frame)
    ff.wF0 = [0; 0; control(3); control(1); control(2); 0];  % torques and forces to base, in INERTIAL frame
    ff.wFm = zeros(6,ff.robot.n_links_joints);               % torques to links, in INERTIAL frame (includes base again)
    % Set generalized forces (F/T in LINK frames)
    ff.tauq0 = zeros(6,1);                                   % torques and forces to base, in LINK frames
    ff.tauqm = [control(4:end)];                             % torques to links, in LINK frames
    
    % Calculate FD on (simulated) real system
    ff.calc_fd();
    
    % Grab accelerations
    if ff.robot.n_q == 1
        r1dd = ff.u0dot_FD(4);
        r2dd = ff.u0dot_FD(5);
        q0dd = ff.u0dot_FD(3);
        q1dd = ff.umdot_FD(1);
        
        % Output derivative of state vector
        dxdt = [r1d; r2d; q0d; q1d; r1dd; r2dd; q0dd; q1dd];
    elseif ff.robot.n_q == 2
        r1dd = ff.u0dot_FD(4);
        r2dd = ff.u0dot_FD(5);
        q0dd = ff.u0dot_FD(3);
        q1dd = ff.umdot_FD(1);
        q2dd = ff.umdot_FD(2);
        % Output derivative of state vector
        dxdt = [r1d; r2d; q0d; q1d; q2d; r1dd; r2dd; q0dd; q1dd; q2dd];
    elseif ff.robot.n_q == 3
        r1dd = ff.u0dot_FD(4);
        r2dd = ff.u0dot_FD(5);
        q0dd = ff.u0dot_FD(3);
        q1dd = ff.umdot_FD(1);
        q2dd = ff.umdot_FD(2);
        q3dd = ff.umdot_FD(3);
        % Output derivative of state vector
        dxdt = [r1d; r2d; q0d; q1d; q2d; q3d; r1dd; r2dd; q0dd; q1dd; q2dd; q3dd];
    end
    t
end

