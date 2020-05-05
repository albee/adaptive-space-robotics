%{ 
Create a spart robot object from a URDF file.
Inputs: URDF filename
Outputs: robot model
%}
function [robot, H, C] = urdf_to_spart_model(filename)
    if nargin == 0
        filename = './urdf/astrobee_planar_1_link.urdf';
    end

    %--- Create robot model ---%
    [robot,robot_keys] = urdf2robot(filename);

    %--- Parameters ---%
    R0=eye(3);
    r0=zeros(3,1);
    qm=zeros(robot.n_q,1);
    u0=zeros(6,1);
    um=zeros(robot.n_q,1);

    %--- Kinematics, required for FD and must be recomputed each time step ---%
    %Kinematics
    [RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
    %Diferential Kinematics
    [Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
    %Velocities
    [t0,tm]=Velocities(Bij,Bi0,P0,pm,u0,um,robot);
    %Inertias in inertial frames
    [I0,Im]=I_I(R0,RL,robot);

    %--- Dynamics Matrices ---%
    %Mass Composite Body matrix
    [M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
    %Generalized Inertia matrix
    [H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
    %Generalized Convective Inertia matrix
    [C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
    
    H = [H0, H0m;
         H0m', Hm];
     
    C = [C0, C0m,
         Cm0, Cm];
end
