clc
clear
close all

% Joint errors
mu = 0; sig = 0.0045 ; num_std = 3;

% % Link lengths
% L0 = 0.27035; o1 = 0.069;
% L1 = 0.36435; o2 = 0.069;
% L2 = 0.37429; o3 = 0.010;
% L3 = 0.22952;
% this needs to be confirmed
% gripper_link = 0.025 + 0.1372;

% Define rotation axes (wrt. arm_mount frame for both the arms)
w1 = [0;0;1]; w2 = [0;1;0]; w3 = [1;0;0];
w4 = w2; w5 = w3; w6 = w2; w7 = w3;
wr = [w1, w2, w3, w4, w5, w6, w7];

% Joint type
joint_type = ['R', 'R', 'R', 'R', 'R', 'R', 'R'];

% Frame origins (wrt. arm_mount frame for both the arms)
% q1 = [0;0;0]; q2 = [o1;0;L0]; q3 = q2; q4 = [o1+L1;0;L0-o2];
% q5 = q4; q6 = [o1+L1+L2;0;L0-o2-o3]; q7 = q6;
% qr = [q1, q2, q3, q4, q5, q6, q7];

% Frame origins (wrt. arm_mount frame for both the arms)
q1 = [0.056; 0.000; 0.011];
q2 = [0.125; 0.000; 0.281];
q3 = [0.227; 0.000; 0.281];
q4 = [0.489; 0.000; 0.213];
q5 = [0.593; -0.001; 0.213];
q6 = [0.863; -0.001; 0.195];
q7 = [0.979; -0.002; 0.195];
qr = [q1, q2, q3, q4, q5, q6, q7];

% Define g_st0 (end point) (Left-Arm)
g_st0_left = [0,0,1,1.213; 0,1,0,-0.002; -1,0,0,0.190; 0,0,0,1.0000];

% Define g_st0 (end point) (Right-Arm)
g_st0_right = [0,0,1,1.213; 0,1,0,-0.002; -1,0,0,0.190; 0,0,0,1.0000];

% Define base transformation of (Right-Arm)
g_base_right = [0.7071, 0.7071, 0.0, 0.025; -0.7071, 0.7071, 0.0, -0.220;
               0.0, 0.0, 1.0, 0.109; 0, 0, 0, 1.0];
          
% Define base transformation of (Left-Arm)
g_base_left = [0.7071, -0.7071, 0.0, 0.025; 0.7071, 0.7071, 0.0, 0.220;
               0.0, 0.0, 1.0, 0.109; 0, 0, 0, 1.0];

% Theta 
% theta_l = [-0.260885,0.794971,-2.60656,1.7124,-1.92283,-1.46809,0.713306];
% theta_r = [0.195445,0.80119,2.62559,1.73094,1.94822,-1.48275,-0.716807];
           
% Load joint solutions
[iksol_l, delimiterOut_l] = importdata('ikfast_sols/ik_sol_left.txt');
[iksol_r, delimiterOut_r] = importdata('ikfast_sols/ik_sol_right.txt');

% Transform joint axes and origins into base frame
wr_l = g_base_left(1:3, 1:3) * wr; 
qr_l = g_base_left(1:3, 1:3) * qr + g_base_left(1:3, 4);
gst0_l = g_base_left * g_st0_left;

wr_r = g_base_right(1:3, 1:3) * wr;
qr_r = g_base_right(1:3, 1:3) * qr + g_base_right(1:3, 4);
gst0_r = g_base_right * g_st0_right;


% Loop through all combinatorics of ik solution pair
tic;
wr_c = [wr_l, wr_r];
qr_c = [qr_l, qr_r];
error_array = zeros(3, size(iksol_l, 1)*size(iksol_r, 1));
count = 1;
% Load joint solutions
js_l = [-0.350, -0.599, -0.418, 0.891, -1.177, 1.795, 0.375];
js_r = [0.129, -0.790, 0.599, 1.360, 1.160, 1.525, -0.557];
for ii = 1:size(iksol_l, 1)
    % theta_l = iksol_l(ii, :);
    theta_l = js_l;
    % Solve forward kinematics (Left-Arm)
    [gst_l, transform_upto_joint_l] = ...
        direct_kin(gst0_l, joint_type, wr_l, qr_l, theta_l);
    for jj = 1:size(iksol_r, 1)
        % theta_r = iksol_r(jj, :);
        theta_r = js_r;
        
        % Solve forward kinematics (Right-Arm)
        [gst_r, transform_upto_joint_r] = ...
            direct_kin(gst0_r, joint_type, wr_r, qr_r, theta_r);
        
        % Compute Jacobian for g_rel
        theta_c = [theta_l, theta_r];
        analytical_jacobian = ...
            relative_jacobian(gst0_l, gst0_r, gst_l, gst_r, transform_upto_joint_l, transform_upto_joint_r, wr_c, qr_c, theta_c);
        [V, D] = eig(analytical_jacobian(1:3, :)*analytical_jacobian(1:3, :)');
        sq_error = max(diag(D));
        error_array(1, count) = sq_error;
        error_array(2, count) = ii;
        error_array(3, count) = jj;
        count = count + 1;
    end
end
toc;

% Get the IK pair that has minimum error
[M, I] = min(error_array(1, :));
th_l = iksol_l(error_array(2, I),:);
th_r = iksol_r(error_array(3, I),:);
fprintf("IK pair corresponding to minimum error in R^3\n");
fprintf("theta-left: \n"); disp(th_l);
fprintf("theta-right: \n"); disp(th_r);
fprintf("Error: %2.6f\n", sqrt(M)*(num_std*sig));