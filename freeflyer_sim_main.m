%%
% Keenan Albee, final project
% Two-link free-flying manipulator adaptive control

% function freeflyer_sim_main()
addpath('./SPART');
addpath('./plot');
clc; close all; clearvars;
fontsize = 25;

tf = 10.0; % specified in create_ref_traj()

%--- Set up simulated real system ---%
filename='./SPART/urdf/astrobee_planar_2_link_grapple.urdf';
[robot] = urdf_to_spart_model(filename);
ff = spart_free_flyer_dynamics(robot);  % this can solve the forward dynamics, but we need the essential matrix

%--- Set up model system ---%
filename='./SPART/urdf/astrobee_planar_2_link.urdf';
[robot] = urdf_to_spart_model(filename);
ff_model = spart_free_flyer_dynamics(robot);  % this can solve the forward dynamics, but we need the essential matrix

num_joints = ff.robot.n_q;
n = 2*(3 + num_joints);
m = 3 + num_joints;

ref_traj = create_ref_traj();
t_des_hist = ref_traj(:, 1);
x_des_hist = ref_traj(:, 2:n+1);
xdd_des_hist = ref_traj(:, n+2:end);

control_hist = [];

adaptive = 0;
Y = analytical_dynamics();

%% Setup
if (adaptive == 1)
    %% Adaptive control
    %Initial conditions
    % state : [r1 r2 q0 q1 r1d r2d q0d q1d]' INERTIAL
    x_0 = zeros(n+1,1);
    tspan = [0, tf]; % seconds
    
    dynamics = @(t, x) freeflyer_dyn_adapt(t, x, ff, ff_model, ref_traj, Y);
    odeOptions = odeset('RelTol',1e-3,'AbsTol',1e-3);
else
    %% Nominal system control
    %Initial conditions
    % state : [r1 r2 q0 q1 r1d r2d q0d q1d]' INERTIAL
    %         [r1 r2 q0 q1 q2 r1d r2d q0d q1d q2d]' INERTIAL
    x_0 = zeros(n,1);
    tspan = [0, tf]; % seconds

    dynamics = @(t, x) freeflyer_dyn(t, x, ff, ff_model, ref_traj, Y);
    odeOptions = odeset('RelTol',1e-3,'AbsTol',1e-3);
end

%% Dynamics
[tvec, state_hist] = ode23(dynamics, tspan, x_0, odeOptions);  % produces [t, n] of state history

% recreate the control history
for i = 1:1:size(tvec,1)
    [x_des, xdd_des] = lookup_ref_traj(tvec(i), ff, ref_traj);
    if adaptive == 1
        ff_model.a_hat = state_hist(i,11);
    end
    control_hist(i, :) = freeflyer_adaptive_control(x_des, state_hist(i,:)', xdd_des, ff, ff_model, Y)';
end

%% Plot state vs time
r1_hist = state_hist(:,1);  % deg
r2_hist = state_hist(:,2);  % deg
q0_hist = state_hist(:,3);  % deg
q1_hist = state_hist(:,4);  % deg
q2_hist = state_hist(:,5);  % deg/s
r1d_hist = state_hist(:,6);  % deg/s
r2d_hist = state_hist(:,7);  % deg/s
q0d_hist = state_hist(:,8);  % deg/s
q1d_hist = state_hist(:,9);
q2d_hist = state_hist(:,10);

r1_des_hist = x_des_hist(:,1);
r2_des_hist = x_des_hist(:,2);
q0_des_hist = x_des_hist(:,3);
q1_des_hist = x_des_hist(:,4);
q2_des_hist = x_des_hist(:,5);

F1_hist = control_hist(:,1);
F2_hist = control_hist(:,2);
tau0_hist = control_hist(:,3);
tau1_hist = control_hist(:,4);
tau2_hist = control_hist(:,5);

%--- Set up animation ---%
% Choose an animation rate
fig = figure('units','normalized','outerposition',[0 0 1 1])
view(225, 75);
hold on;
axis equal;
axis([-5, 1, -3, 3, -2, 2]*1.0)

plot_dt = 0.05;  % period for plot frames, s
t = 0:plot_dt:tf;
cntr = 1;
frames = [];
for i = 1:1:size(tvec,1)
    if tvec(i) > t(cntr)
        cntr = cntr+1;
        frames = [frames; i];
    end
end

ff.plot_from_history_FK3(fig, state_hist(frames,:), control_hist(frames,:));


% Default plotting
figure
hold on
plot(tvec, r1_hist, '-r', 'linewidth', 2)
plot(tvec, r2_hist,'-b', 'linewidth', 2)
plot(t_des_hist, r1_des_hist, '--r', 'linewidth', 2)
plot(t_des_hist, r2_des_hist, '--b', 'linewidth', 2)
legend('$r_1$', '$r_2$', '$r_{1,des}$', '$r_{2,des}$','Interpreter', 'latex', 'FontSize', fontsize);
grid on;
set(gca, 'FontSize', fontsize-5)
xlabel('$t$ [$s$]', 'Interpreter', 'latex', 'FontSize', fontsize)
ylabel('Position [$m$]', 'Interpreter', 'latex', 'FontSize', fontsize)
title('2-Link Manipulator Position Performance','Interpreter', 'latex', 'FontSize', fontsize)

figure
hold on
plot(tvec, q0_hist, '-r', 'linewidth', 2)
plot(tvec, q1_hist, '-black', 'linewidth', 2)
plot(tvec, q2_hist, '-b', 'linewidth', 2)
plot(t_des_hist, q0_des_hist, '--r', 'linewidth', 2)
plot(t_des_hist, q1_des_hist, '--black', 'linewidth', 2)
plot(t_des_hist, q2_des_hist, '--b', 'linewidth', 2)
legend('$q_0$', '$q_1$', '$q_2$', '$q_{0,des}$', '$q_{1,des}$', '$q_{2,des}$', 'Interpreter', 'latex', 'FontSize', fontsize);
grid on;
set(gca, 'FontSize', fontsize-5)
xlabel('$t$ [$s$]', 'Interpreter', 'latex', 'FontSize', fontsize)
ylabel('Angle [$rad$]', 'Interpreter', 'latex', 'FontSize', fontsize)
title('2-Link Manipulator Angular Performance','Interpreter', 'latex', 'FontSize', fontsize)

figure
hold on
plot(tvec(1:end-20), F1_hist(1:end-20), '-r', 'linewidth', 2)
plot(tvec(1:end-20), F2_hist(1:end-20), '-b', 'linewidth', 2)
plot(tvec(1:end-20), tau0_hist(1:end-20), '-black', 'linewidth', 2)
plot(tvec(1:end-20), tau1_hist(1:end-20), '-g', 'linewidth', 2)
plot(tvec(1:end-20), tau2_hist(1:end-20), '-c', 'linewidth', 2)
legend('$F_1$', '$F_2$', '$\tau_0$', '$\tau_{1}$', '$\tau_{2}$', 'Interpreter', 'latex', 'FontSize', fontsize);
grid on;
set(gca, 'FontSize', fontsize-5)
xlabel('$t$ [$s$]', 'Interpreter', 'latex', 'FontSize', fontsize)
ylabel('Input [$N$, $N-m$]', 'Interpreter', 'latex', 'FontSize', fontsize)
title('2-Link Manipulator Input History','Interpreter', 'latex', 'FontSize', fontsize)

if adaptive
    figure
    hold on
    a1_hist = state_hist(:,11);     
    plot(tvec(1:end-10), a1_hist(1:end-10),'-black', 'linewidth', 2)
    plot(tvec(1:end-10), 8*ones(size(tvec,1)-10),'--black', 'linewidth', 2)
    ylim([-5,10]);
    legend('$\hat{m}_3$', '$m_3$', 'Interpreter', 'latex', 'FontSize', fontsize);
    xlabel('$t$ [$s$]', 'Interpreter', 'latex', 'FontSize', fontsize)
    ylabel('mass [$kg$]', 'Interpreter', 'latex', 'FontSize', fontsize)
    title('Estimated End Effector Mass','Interpreter', 'latex', 'FontSize', fontsize)
end
% end