%{
This is a wrapper class around the SPART dynamics functions. This makes
obtaining some of the values simpler, and makes handling all the parameters
and states of a free-flying robot easier.

Keenan Albee, 2020
%}

classdef spart_free_flyer_dynamics < handle
    properties
        % NOTE: [r0, R0, qm, u0, um]' gives the entire robot state
        % Robot state, e.g. 13D for 2 link robot
        R0 = [];  % Robot base orientation relative to inertial frame
        r0 = [];  % Robot base position relative to inertial frame
        qm = [];  % Robot joint angles, rad?
        u0 = [];  % Robot base rates wrt I, [w_0; r_dot_I], in base and inertial frames, respectively
        um = [];  % Robot joint rates, [q_dot1_0; ... q_dotn_0] in local frames
        
        % Kinematics
        RJ = [];  % Joint 3x3 rotation matrices in inertial frame as a [3x3xn] matrix.
        RL = [];  % Links 3x3 rotation matrices in inertial frame as a [3x3xn] matrix.
        rJ = [];  % Positions of the joints, in inertial frame as a [3xn] matrix.
        rL = [];  % Positions of the links, in inertial frame as a [3xn] matrix
        e = [];   % Joint rotation/sliding axes, in inertial frame as a [3xn] matrix
        g = [];   % Vector from the ith joint frame to the ith link frame, in inertial frame as a [3xn] matrix.
        
        % Velocity Kinematics
        Bij = [];  % Twist-propagation matrix (for manipulator i>0 and j>0) – as a [6x6xn] matrix
        Bi0 = [];  % Twist-propagation matrix (for i>0 and j=0) – as a [6x6xn] matrix
        P0 = [];  % Base-link twist-propagation “vector” – as a [6x6] matrix
        pm = [];  % Manipulator twist-propagation “vector” – as a [6xn] matrix.
        
        t0 = [];  % Base-link twist [omega,rdot], projected in the inertial CCS – as a [6x1] matrix
        tm = [];  % Manipulator twist [omega,rdot], projected in the inertial CCS – as a [6xn] matrix.
        
        % Inertial properties
        I0= [];  % Base-link inertia matrix, projected in the inertial CCS – as a [3x3] matrix.
        Im = [];  % Links inertia matrices, projected in the inertial CCS – as a [3x3xn] matrix
        robot = [];  % Robot parameters
        
        % Genrealized force/wrench inputs
        wF0 = [];  % Wrench acting on the base-link center-of-mass [t;f], projected in the inertial frame – as a [6x1] matrix.
        wFm = [];  % Wrench acting on the links center-of-mass [t;f], projected in the inertial frame – as a [6xn] matrix.
        tauq0 = []; % Base-link forces [t;f]. The torque t is projected in the base frame, while the force f is projected in the inertial frame – [6x1]
        tauqm = []; % Joint forces/torques in their local frames – as a [n_qx1] matrix.
        
        % Generalized accelerations
        u0dot_FD = [];  % [6x1]
        umdot_FD = [];  % [nx1]
        
        % Mass composite body matrices, used for getting H and C
        M0_tilde = []
        Mm_tilde = []
        
        % Custom control history for plotting
        control;  % [F_1_BASE; F_2_BASE; T_0; T_1; T_2];
        a_hat;
        a_hatd;
    end
    
    methods
        % Constructor
        function self = spart_free_flyer_dynamics(robot)
        % Given input forces and torques to the base (tau0)
        % and the joints (taum) and the current system configuration,
        % find the resulting accelerations of the system generalized
        % coordinates. Relies on the SPART library.
        
        %--- Set robot state ---%
        self.R0=eye(3);             % base rotation matrix
        self.r0=zeros(3,1);         % base position
        self.qm=zeros(robot.n_q,1); % joint positions
        self.u0=zeros(6,1);         % base linear and angular velocities
        self.um=zeros(robot.n_q,1); % joint velocities
        
        %--- Set robot wrenches ---%
        self.wF0=zeros(6,1);
        self.wFm=zeros(6,robot.n_links_joints);
        self.tauq0=zeros(6,1);
        self.tauqm=zeros(robot.n_q,1);
        
        %--- Set robot parameters ---%
        self.robot = robot;
        end
        
        function self = calc_fk(self)  % Must have an updated robot kinematic state
            [self.RJ, self.RL, self.rJ, self.rL, self.e, self.g] =...
                Kinematics(self.R0, self.r0, self.qm, self.robot);
        end
        
        function self = calc_vel_kinematics(self) % Must have an updated robot kinematic state, with FK 
            [self.Bij, self.Bi0, self.P0, self.pm] =...
                DiffKinematics(self.R0, self.r0, self.rL, self.e, self.g, self.robot);
        end
        
        function self = calc_twist(self)  % Must have an updated robot velocity state, with velocity kinematics
            [self.t0, self.tm] =...
                Velocities(self.Bij, self.Bi0, self.P0, self.pm, self.u0, self.um, self.robot);
        end
        
        function self = calc_inertias(self)  % Must have an updated robot kinematic state, with FK
            [self.I0, self.Im] = I_I(self.R0, self.RL, self.robot);
        end
         
        % Find the generalized accelerations for the current state of the robot
        function self = calc_fd(self)
            self.calc_fk();
            self.calc_vel_kinematics();
            self.calc_twist();
            self.calc_inertias();
            
            [self.u0dot_FD, self.umdot_FD] =...
                FD(self.tauq0, self.tauqm, self.wF0, self.wFm, self.t0, self.tm,...
                self.P0, self.pm, self.I0, self.Im, self.Bij, self.Bi0, self.u0,...
                self.um, self.robot);
        end

        % Forward propagate the state for a single euler step
        function self = euler_integrate(self, dt)
            [self.r0, self.R0, self.qm, self.u0, self.um] =...
                euler_integrate(self.u0dot_FD, self.umdot_FD, self.r0, self.R0, self.qm, self.u0, self.um, dt);

            % Update the robot kinematics
            self.calc_fk();
        end
        
        % Get the GIM
        function H = calc_GIM(self)
            self.calc_fk();
            self.calc_vel_kinematics();
            self.calc_twist();
            
            [self.I0, self.Im] = I_I(self.R0, self.RL, self.robot);  % inertias in inertial frame
            [self.M0_tilde, self.Mm_tilde] = MCB(self.I0, self.Im, self.Bij, self.Bi0, self.robot);  % mass composite body matrix
            [H0, H0m, Hm] = GIM(self.M0_tilde, self.Mm_tilde, self.Bij, self.Bi0, self.P0, self.pm, self.robot);
            H = [H0, H0m;
                 H0m', Hm];
        end
        
        % Get the CIM
        function C = calc_CIM(self)
            self.calc_fk();
            self.calc_vel_kinematics();
            self.calc_twist();

            [self.I0, self.Im] = I_I(self.R0, self.RL, self.robot);  % inertias in inertial frame
            [self.M0_tilde, self.Mm_tilde] = MCB(self.I0, self.Im, self.Bij, self.Bi0, self.robot);  % mass composite body matrix
            [C0, C0m, Cm0, Cm] = CIM(self.t0, self.tm, self.I0, self.Im, self.M0_tilde, self.Mm_tilde, self.Bij, self.Bi0, self.P0, self.pm, self.robot);
            C = [C0, C0m;
                 Cm0, Cm];
        end
        
        % Calculate Center of Mass
        function COM = COM(self)
            COM = Center_of_Mass(self.r0, self.rL, self.robot);
        end
        
        function fig = plot_FK(self, fig)
            figure(fig)
            cla(fig)

            COM_I = Center_of_Mass(self.r0, self.rL, self.robot);
            r_I = [self.r0, self.rJ];

            scatter(r_I(1,:),r_I(2,:),'r','filled');  % plot joints
            plot(r_I(1,:),r_I(2,:),'Linewidth',2);  % plot links

%             theta = self.% get better rectangle plot!
            % do a 3d test
            % find best way to plug into ROS??
            
%             rectangle('Position',[r_I(1:2,1)'-.5 1 1])  % plot base square
            scatter(COM_I(1,:),COM_I(2,:),50,'bl','filled');  % plot COM
        end
        
        function fig = plot_FK3(self, fig)
            figure(fig)
            cla(fig)

            COM_I = Center_of_Mass(self.r0, self.rL, self.robot);
            r_I = [self.r0, self.rJ];

            scatter3(r_I(1,:),r_I(2,:),r_I(3,:),'black','filled');  % plot joints            
            plot3(r_I(1,:),r_I(2,:),r_I(3,:),'black','Linewidth',2);  % plot links
            scatter3(r_I(1,end),r_I(2,end),r_I(3,end),350,'black','filled');  % plot joints
  
            CLR = [0, 0, 1];
            ALPHA = .8;
            l = self.robot.joints(1).T(1,4)*2;  % double first link dist
            plot_cube(self.r0, self.R0, l, CLR, ALPHA);
            
            scatter3(COM_I(1,:),COM_I(2,:),COM_I(3,:),50,'bl','filled');  % plot COM
            
            self.control'
            quiver3(self.r0(1), self.r0(2), self.r0(3), self.control(1)/30.0, self.control(2)/30.0, 0,...
                'Color', 'black', 'LineWidth', 2.0, 'AutoScale', 'on');
        end
        
        function fig = plot_from_history_FK3(self, fig, state_hist, control_hist)
            for i = 1:1:size(state_hist,1)
                % set internal state
                x = state_hist(i,:)';  % INERTIAL
                r1 = x(1);
                r2 = x(2);
                q0 = x(3);
                
                self.R0 = eul2rotm([q0, 0, 0]);
                self.r0 = [r1; r2; 0];
                self.qm = [];
                for j = 1:1:self.robot.n_q 
                    self.qm = [self.qm; x(3+j)];
                end
                self.calc_fk();
                
                self.control = control_hist(i,:)';
                
                % plot
                fig = self.plot_FK3(fig);
                pause(0.05);
                
                frame(i)=getframe(fig);  % for video
            end
            
            % Make video
            v = VideoWriter('blargh11.avi');
            v.FrameRate=15;
            open(v);
            writeVideo(v,frame);
            close(v);
        end
    end
end