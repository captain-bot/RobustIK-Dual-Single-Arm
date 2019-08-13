clc
clear
close all

% Joint type
joint_type = ['R', 'R', 'R', 'R', 'R', 'R', 'R'];

% Axes Description for each joint in the reference configuration
w1r = [0;0;1]; w2r = [0;1;0]; w3r = [1;0;0]; w4r = [0;1;0];
w5r = [1;0;0]; w6r = [0;1;0]; w7r = [1;0;0]; 
wr = [w1r w2r w3r w4r w5r w6r w7r];

% Choice of points on the axis for each joint in the reference
% configuration
q1r = [0.056; 0.000; 0.011]; q2r = [0.125; 0.000; 0.281];
q3r = [0.227; 0.000; 0.281]; q4r = [0.489; 0.000; 0.213];
q5r = [0.593; -0.001; 0.213]; q6r = [0.863; -0.001; 0.195];
q7r = [0.979; -0.002; 0.195]; 
qr = [q1r q2r q3r q4r q5r q6r q7r];

% gst_0 for left and right arms wrt their respective amr_mount frames
% will be exactly similar to each other
p_st0 = [1.213; -0.002; 0.190];
R_st0 = [0, 0, 1; 0, 1, 0; -1, 0, 0];
last_line = [zeros(1,3) 1];
gst0 = [R_st0 p_st0; last_line];

% Transformations from arm_mount to base frame for left and right arm
T_base2leftarmmount = [0.707, -0.707, 0, 0.025;
                       0.707, 0.707, 0, 0.220;
                       0, 0, 1, 0.109;
                       0, 0, 0, 1];

T_base2rightarmmount = [0.707, 0.707, 0, 0.025;
                       -0.707, 0.707, 0,-0.220;
                        0, 0, 1, 0.109;
                        0, 0, 0, 1];

% Transfrm joint axes and origin locations into base frame
wr_l = T_base2leftarmmount(1:3, 1:3) * wr; 
qr_l = T_base2leftarmmount(1:3, 1:3) * qr + T_base2leftarmmount(1:3, 4);
gst0_l = T_base2leftarmmount * gst0;

wr_r = T_base2rightarmmount(1:3, 1:3) * wr;
qr_r = T_base2rightarmmount(1:3, 1:3) * qr + T_base2rightarmmount(1:3, 4);
gst0_r = T_base2rightarmmount * gst0;

% Load joint solutions
js_l = [-1.11976, -0.637279, 0.797197, 1.74029, -1.87207, 2.07934, 1.06616];
js_r = [0.830758, -0.551991, -0.417708, 1.39497, 1.67068, 2.07934, -1.03666];

% Solve forward kinematics for left and right arms
[gst_l, transform_upto_joint_l] = direct_kin(gst0_l, joint_type, wr_l, qr_l, js_l);
[gst_r, transform_upto_joint_r] = direct_kin(gst0_r, joint_type, wr_r, qr_r, js_r);

% Compute desired relative pose of the end-effectors
g_relative_desired = gst_l\gst_r;

% Compute relative analytical jacobian
wr_c = [wr_l, wr_r];
qr_c = [qr_l, qr_r];
theta_c = [js_l, js_r];
analytical_jacobian = relative_jacobian(gst0_l, gst0_r, gst_l, gst_r, transform_upto_joint_l, transform_upto_joint_r, wr_c, qr_c, theta_c);
