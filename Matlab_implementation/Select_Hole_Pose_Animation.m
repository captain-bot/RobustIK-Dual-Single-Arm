% 1. This piece of code computes desired peg configuration, given the hole
% configuration wrt base frame. Code also returns transformations of
% left_gripper and right_gripper poses with the corresponding arm_mount
% frames which then can be used for IK-Fast input arguments to compute
% inverse kinematics solutions.
%
% 2. Further, this piece of code also shows animation of peg-in-hole
% assembly using T_base2rightgripper and computed T_base2leftgripper
%
% Date: July 14th, 2019

clc;
clear;
close all;

%//////////////////////////////////////////////////////////
%///////////////////// User Input /////////////////////////
%//////////////////////////////////////////////////////////
% Pose right_gripper frame wrt base frame
p_base2rightgripper = [0.787; -0.003; 0.459];
R_base2rightgripper = quat2rotm([-0.057, 0.093, 0.717, 0.688]);
T_base2rightgripper = [R_base2rightgripper, p_base2rightgripper; 0, 0, 0, 1];

% Peg and hole information
peg.radius = 0.020; hole.radius = 0.030;
peg.length = 0.050;  hole.length = 0.050;

%///////////////////////////////////////////////////////////////////
%////////////////// Robot parameters ///////////////////////////////
%///////////////////////////////////////////////////////////////////
% Pose of right_arm_mount frame wrt base frame
p_base2rightarmmount = [0.025; -0.220; 0.109];
R_base2rightarmmount = [0.707, 0.707, 0; -0.707, 0.707, 0; 0, 0, 1];
T_base2rightarmmount = [R_base2rightarmmount, p_base2rightarmmount; 0, 0, 0, 1];

% Pose right_gripper frame wrt right_arm_mount frame
T_rightarmmount2rightgripper = T_base2rightarmmount\T_base2rightgripper;

% Pose of left_arm_mount frame wrt base frame
p_base2leftarmmount = [0.025; 0.220; 0.109];
R_base2leftarmmount = [0.707, -0.707, 0; 0.707, 0.707, 0; 0, 0, 1];
T_base2leftarmmount = [R_base2leftarmmount, p_base2leftarmmount; 0, 0, 0, 1];

%////////////////////////////////////////////////////////////////////
%////////////////// Compute Transformations /////////////////////////
%///////////////////////////////////////////////////////////////////
% Compute transformation of hole frame wrt base frame
T_base2hole = [R_base2rightgripper, p_base2rightgripper + hole.length * R_base2rightgripper(:, 3); 0, 0, 0, 1];

% Compute transformation of peg frame wrt base frame
p_base2peg = p_base2rightgripper + 3* hole.length * R_base2rightgripper(:, 3);
R_base2peg = [R_base2rightgripper(:, 1), -R_base2rightgripper(:, 2), -R_base2rightgripper(:, 3)];
T_base2peg = [R_base2peg, p_base2peg; 0, 0, 0, 1];

% Compute transformation of left_gripper frame wrt base frame
T_base2leftgripper = [R_base2peg, p_base2peg - peg.length*R_base2peg(:, 3); 0, 0, 0, 1];

% Compute transformation of left_gripper frame wrt left_arm_mount frame
T_leftarmmount2leftgripper = T_base2leftarmmount\T_base2leftgripper;

%/////////////////////////////////////////////////////////////////////
%///////////// Print desired transformations /////////////////////////
%/////////////////////////////////////////////////////////////////////
fprintf("Computed::T_base2hole: \n"); disp(T_base2hole);
fprintf("Computed::T_base2peg: \n"); disp(T_base2peg);

fprintf("T_leftarmmount2leftgripper: \n"); disp(T_leftarmmount2leftgripper);
fprintf("p: "); disp(T_leftarmmount2leftgripper(1:3, 4)');
fprintf("q: "); disp(rotm2quat(T_leftarmmount2leftgripper(1:3, 1:3)));

fprintf("T_rightarmmount2rightgripper: \n"); disp(T_rightarmmount2rightgripper);
fprintf("p: "); disp(T_rightarmmount2rightgripper(1:3, 4)');
fprintf("q: "); disp(rotm2quat(T_rightarmmount2rightgripper(1:3, 1:3)));

%////////////////////////////////////////////////////////////////////////
%//////////// Finally visualize the hole pose and peg motion ////////////
%////////////////////////////////////////////////////////////////////////
animate_peg_hole(T_base2peg, T_base2hole, peg, hole);
