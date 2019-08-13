% This is a test code with Baxter robot. The final transformation that we
% get is with respect to "<sid>_arm_mount" frame.

clc
clear all
close all

% Joint types for the manipulator
type_joint = ['R'; 'R'; 'R'; 'R'; 'R'; 'R'; 'R'];

% Axes Description for each joint in the reference configuration

w1r = [0;0;1];
w2r = [0;1;0];
w3r = [1;0;0];
w4r = [0;1;0];
w5r = [1;0;0];
w6r = [0;1;0];
w7r = [1;0;0];
wr = [w1r w2r w3r w4r w5r w6r w7r];

% Choice of points on the axis for each joint in the reference
% configuration

q1r = [0.056; 0.000; 0.011];
q2r = [0.125; 0.000; 0.281];
q3r = [0.227; 0.000; 0.281];
q4r = [0.489; 0.000; 0.213];
q5r = [0.593; -0.001; 0.213];
q6r = [0.863; -0.001; 0.195];
q7r = [0.979; -0.002; 0.195];
qr = [q1r q2r q3r q4r q5r q6r q7r];

% Transformation for tool frame in reference configuration

p_st0 = [1.213; -0.002; 0.190];
R_st0 = [0, 0, 1; 0, 1, 0; -1, 0, 0];
last_line = [zeros(1,3) 1];
gst0 = [R_st0 p_st0; last_line];

% Input the choice of configuration and rate of change of configuration

% theta = pi/3*ones(6,1);
% theta = 0*ones(7,1);
% theta = [0; pi/3; pi/3; pi/3; pi/3; pi/3];
theta = [0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
thetadot = ones(7,1);

% Direct Position Kinematics

[gst, transform_upto_joint] = direct_kin(gst0, type_joint, wr, qr, theta);
gst

%% Check with the formula from MLS book
% p1 = -sin(theta(1))*(L1*cos(theta(2)) + L2*cos(theta(2)+theta(3)));
% p2 = cos(theta(1))*(L1*cos(theta(2)) + L2*cos(theta(2)+theta(3)));
% p3 = L0 - L1*sin(theta(2)) - L2*sin(theta(2)+theta(3));
% p = [p1;p2;p3];

% Direct Velocity Kinematics
[spatial_jac, spatial_vel_st] = velocity_direct_kin(gst0, type_joint, wr, qr, theta, thetadot);