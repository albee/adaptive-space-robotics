function Y = analytical_dynamics()
    filename='./SPART/urdf/astrobee_planar_2_link.urdf';
    [robot] = urdf_to_spart_model(filename);
    ff = spart_free_flyer_dynamics(robot);

    m3 = sym('m3',[1,1],'real');
    ff.robot.links(3).mass = m3;  % unknown mass

    %Base-link position
    r1=sym('r1', [1], 'real');
    r2=sym('r2', [1], 'real');
    q0=sym('q0', [1], 'real');
    q1=sym('q1',[1],'real');
    q2=sym('q2',[1],'real');

    r1d=sym('r1d', [1], 'real');
    r2d=sym('r2d', [1], 'real');
    w0=sym('w0', [1], 'real');
    q1d=sym('q1d',[1],'real');
    q2d=sym('q2d',[1],'real');
    xd = [r1d; r2d; w0; q1d; q2d];
    
    r1d_r=sym('r1d_r', [1], 'real');
    r2d_r=sym('r2d_r', [1], 'real');
    w0_r=sym('w0_r', [1], 'real');
    q1d_r=sym('q1d_r',[1],'real');
    q2d_r=sym('q2d_r',[1],'real');
    xd_r = [r1d_r; r2d_r; w0_r; q1d_r; q2d_r];

    r1dd=sym('r1dd', [1], 'real');
    r2dd=sym('r2dd', [1], 'real');
    w0d=sym('w0d', [1], 'real');
    q1dd=sym('q1dd',[1],'real');
    q2dd=sym('q2dd',[1],'real');
    xdd_r = [r1dd; r2dd; w0d; q1dd; q2dd];

    ff.r0=[r1, r2, 0]';
    ff.R0 = Angles321_DCM([0, 0, q0])';
    ff.qm=[q1; q2];

    ff.calc_fk()
    ff.calc_vel_kinematics();

    ff.u0=[0, 0, w0, r1d, r2d, 0]';       % base linear and angular velocities
    ff.um=[q1d; q2d]; % joint velocities

    ff.calc_twist();

    H = ff.calc_GIM;
    C = ff.calc_CIM;

    % convert to Keenan convention
    C = [C(4,:);   % rd1
         C(5,:);   % rd2
         C(3,:);   % w_z
         C(7,:);   % qd1
         C(8,:)];  % qd2
    C = [C(:,4), C(:,5), C(:,3), C(:,7), C(:,8)]  % [F1 F2 tau0 tau1 tau2] 

    H = [H(4,:);   % rd1
         H(5,:);   % rd2
         H(3,:);   % w_z
         H(7,:);   % qd1
         H(8,:)];  % qd2
    H = [H(:,4), H(:,5), H(:,3), H(:,7), H(:,8)]  % [F1 F2 tau0 tau1 tau2]

    % C_regress = collect(C, m3);
    % H_regress = collect(H, m3);
    Y = collect(H*xdd_r + C*xd_r, m3);

    Y = symfun(Y, [r1, r2, q0, q1, q2, r1d, r2d, w0, q1d, q2d, r1d_r, r2d_r, w0_r, q1d_r, q2d_r, r1dd, r2dd, w0d, q1dd, q2dd, m3]);
    Y = matlabFunction(Y);
%     Y(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
end
