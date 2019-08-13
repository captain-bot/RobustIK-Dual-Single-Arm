% Script file to test the codes for direct position kinematics and direct 
% velocity kinematics for serial chain manipulators

% Test using the Elbow manipulator. The reference configuration is the
% configuration with theta = 0;

clear all;
clc;


% Link Lengths for the Elbow manipulator

L0 = 1;
L1 = 1;
L2 = 0.5;

% Joint types for the manipulator

type_joint = ['R'; 'R'; 'R'; 'R'; 'R'; 'R'];

% Axes Description for each joint in the reference configuration

w1r = [0;0;1];
w2r = [-1;0;0];
w3r = [-1;0;0];
w4r = [0;0;1];
w5r = [-1;0;0];
w6r = [0;1;0];
wr = [w1r w2r w3r w4r w5r w6r];

% Choice of points on the axis for each joint in the reference
% configuration

q1r = [0;0; L0];
q2r = q1r;
q3r = [0;L1;L0];
q4r = [0; L1+L2; L0];
q5r = q4r;
q6r = q4r;
qr = [q1r q2r q3r q4r q5r q6r];

% Transformation for tool frame in reference configuration

p_st0 = [0; L1 + L2; L0];
R_st0 = eye(3,3);
last_line = [zeros(1,3) 1];
gst0 = [R_st0 p_st0; last_line];

% Input the choice of configuration and rate of change of configuration

theta = pi/3*ones(6,1);
%theta = 0*ones(6,1);
%theta = [0; pi/3; pi/3; pi/3; pi/3; pi/3];
thetadot = ones(6,1);

% Direct Position Kinematics

[gst, transform_upto_joint] = direct_kin(gst0, type_joint, wr, qr, theta);
gst

%% Check with the formula from MLS book
% p1 = -sin(theta(1))*(L1*cos(theta(2)) + L2*cos(theta(2)+theta(3)));
% p2 = cos(theta(1))*(L1*cos(theta(2)) + L2*cos(theta(2)+theta(3)));
% p3 = L0 - L1*sin(theta(2)) - L2*sin(theta(2)+theta(3));
% p = [p1;p2;p3];

% Direct Velocity Kinematics
[spatial_jac, spatial_vel_st] = velocity_direct_kin(gst0, type_joint, wr, qr, theta, thetadot)



