function dydt = problem3_dyn(t, y)
% 2-link manipulator dynamics (no gravity). Adaptive control.
%
% Inputs:
% t         (1x1) time vector
% y         (4x1) state vector [q1 q2 q1d q2d]'
%
% Output:
% dydt      (4x1) derivative of state vector [q1d, q2d, q1dd, q2dd]'

% Unpack state vec
q1 = y(1);
q2 = y(2);
q1d = y(3);
q2d = y(4);
a1_hat = y(5);
a2_hat = y(6);
a3_hat = y(7);
a4_hat = y(8);
a_hat = y(5:8);

% Constants
m1 = 1;
l1 = 1;
me = 2;
delta_e = 30;
I1 = .12;
lc1 = 0.5;
Ie = 0.25;
lce = 0.6;

a1 = I1 + m1*lc1^2 + Ie + me*lce^2 + me*l1^2;
a2 = Ie + me*lce^2;
a3 = me*l1*lce*cosd(delta_e);
a4 = me*l1*lce*sind(delta_e);

% Dynamics params
H11 = a1 + 2*a3*cosd(q2) + 2*a4*sind(q2);
H21 = a2 + a3*cosd(q2) + a4*sind(q2);
H12 = H21;
H22 = a2;
h = a3*sind(q2) - a4*cosd(q2);

%% Desired traj.

%---Adaptive---
%1
% q1_des = (1 - exp(-t))*180/pi;
% q1d_des = exp(-t)*180/pi;
% q1dd_des = -exp(-t)*180/pi;
% 
% q2_des = 2*(1 - exp(-t))*180/pi;
% q2d_des = 2*exp(-t)*180/pi;
% q2dd_des = -2*exp(-t)*180/pi;

%2
q1_des = (1 - cos(2*pi*t))*180/pi;
q1d_des = 2*pi*sin(2*pi*t)*180/pi;
q1dd_des = 4*pi^2*cos(2*pi*t)*180/pi;

q2_des = 2*(1 - cos(2*pi*t))*180/pi;
q2d_des = (4*pi)*sin(2*pi*t)*180/pi;
q2dd_des = (8*pi^2)*cos(2*pi*t)*180/pi;
%---Adaptive---

%% Control
lambda = 20.0;

%---Adaptive---
% control law, tau = Y*a_hat - Kd*s
q1dr = q1d_des - lambda*(q1 - q1_des);
q2dr = q2d_des - lambda*(q2 - q2_des);
q1ddr = q1dd_des - lambda*(q1d - q1d_des); 
q2ddr = q2dd_des - lambda*(q2d - q2d_des);
s1 = q1d - q1dr;
s2 = q2d - q2dr;
s = [s1; s2];

Kd = [1500, 0;
      0, 1500];
  
Y = [ q1ddr, q2ddr,...
    (2*q1ddr + q2ddr)*cosd(q2) - (q2d*q1dr + q1d*q2dr + q2d*q2dr)*sind(q2),...
        (2*q1ddr + q2ddr)*sind(q2) + (q2d*q1dr + q1d*q2dr + q2d*q2dr)*cosd(q2);
  0,     q1ddr + q2ddr,...
            q1ddr*cosd(q2) + q1d*q1dr*sind(q2),...
                q1ddr*sind(q2) - q1d*q1dr*cosd(q2)];

control = Y*a_hat - Kd*s;
      
% H11_hat = a1_hat + 2*a3_hat*cosd(q2) + 2*a4_hat*sind(q2);
% H21_hat = a2_hat + a3_hat*cosd(q2) + a4_hat*sind(q2);
% H12_hat = H21_hat;
% H22_hat = a2_hat;
% h_hat = a3_hat*sind(q2) - a4_hat*cosd(q2);
% 
% H_hat = [H11_hat, H12_hat;
%          H21_hat, H22_hat];
%  
% C_hat = [-h_hat*q2d, -h_hat*(q1d + q2d);
%          h_hat*q1d,   0];
% 
% control = H_hat*[q1ddr; q2ddr] + C_hat*[q1dr; q2dr] - [300*s1; 300*s2];



% adaptation law, ad_hat = -gamma*Y'*s
gamma = diag([0.03, 0.05, 0.1, 0.3]*0.1);
                
ad_hat = -gamma*Y'*s;

a1d_hat_out = ad_hat(1);
a2d_hat_out = ad_hat(2);
a3d_hat_out = ad_hat(3);
a4d_hat_out = ad_hat(4);

%---Adaptive---

%% Dynamics
q1d_out = q1d;
q2d_out = q2d;

H = [H11, H12;
     H21, H22];
 
C = [-h*q2d, -h*(q1d + q2d);
     h*q1d,   0];
 
Tau = control;

qdd_out = (H)\(Tau - C*[q1d; q2d]);

q1dd_out = qdd_out(1);
q2dd_out = qdd_out(2);

% Output derivative of state vector
dydt = [q1d_out; q2d_out; q1dd_out; q2dd_out; a1d_hat_out; a2d_hat_out; a3d_hat_out; a4d_hat_out];

end

