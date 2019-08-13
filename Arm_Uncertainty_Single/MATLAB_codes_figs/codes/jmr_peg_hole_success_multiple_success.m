clc;
clear all;
close all;

% This code generates plots of success rates with varying clearance between
% peg and hole for robust and worst IKs seperately.
% We considered following left_end_effector poses expressed in base frame.

% Range of radial clearance between peg and hole
rad_clearance = 0.003:0.0005:0.007;

% Pose1: pd = [0.616, 0.077, 0.402], qd = [0.6839, 0.7174, 0.0799, -0.1064]
% th_star = [-0.9357, -0.9376, 0.6222, 1.749, -1.504, 2.079, -2.589]
% th_minus = [0.277, -0.504, -1.219, 1.670, -0.598, 1.199, -3.041]
% For sigma = 0.0035, obj_star = 0.0091, obj_min = 0.0106
success_star1 = [71.64, 79.47, 85.59, 90.14, 93.55 96.31, 97.62, 98.61, 99.28];
success_minus1 = [57.92, 65.89, 74.10, 80.06, 85.28, 89.46, 92.38, 94.66, 96.29];

% Pose2: pd = [0.626, 0.144, 0.234], qd = [0.6839, 0.7174, 0.0799, -0.1064]
% th_star = [-0.7375, -0.5573, 0.4980, 1.5684, -1.4575, 2.0793, -2.3384]
% th_minus = [0.382, 0.4945, -1.6969, 1.4733, 0.5618, 1.3610, 2.8022]
% For sigma = 0.0035, obj_star = 0.0089, obj_min = 0.0104
success_star2 = [73.64, 81.41, 87.29, 91.68, 94.52, 96.72, 98.02, 98.95, 99.26];
success_minus2 = [59.47, 68.46, 75.17, 82.23, 86.63, 90.85, 93.64, 95.63, 97.15];

% Pose3: pd = [0.710, 0.060, 0.210], qd = [0.6839, 0.7174, 0.0799, -0.1064]
% th_star = [-0.9439, -0.3523, 0.6425, 1.2968, -1.8410, 2.0793, -2.5654]
% th_minus = [0.0749, 0.5858, -1.8814, 1.2204, 0.705, 1.368, 2.817]
% For sigma = 0.0035, obj_star = 0.0094, obj_min = 0.011
success_star3 = [65.87, 73.82, 81.60, 87.10, 91.03, 94.00, 95.87, 97.59, 98.37];
success_minus3 = [56.99, 65.22, 72.28, 79.60, 84.91, 88.77, 92.37, 94.22, 96.21];

% Pose4: pd = [0.701, 0.101, 0.380], qd = [0.6839, 0.7174, 0.0799, -0.1064]
% th_star = [-0.7727, 0.7447, 2.6968, 1.4036, 1.6495, 2.0793, 2.2365]
% th_minus = [0.0500, -0.5188, -0.9489, 1.3399, -0.7199, 1.4197, -3.0194]
% For sigma = 0.0035, obj_star = 0.0095, obj_min = 0.01089
success_star4 = [70.33, 78.55, 85.54, 89.41, 93.11, 95.35, 97.11, 98.50, 98.98];
success_minus4 = [59.25, 67.81, 76.07, 81.86, 86.66, 90.68, 93.90, 95.73, 97.55];

% Pose5: pd = [0.776, 0.082, 0.273], qd = [0.6839, 0.7174, 0.0799, -0.1064]
% th_star = [-0.890, -0.305, 0.722, 0.982, -2.020, 2.079, -2.866]
% th_minus = [-0.041, 0.206, -1.625, 0.904, 0.227, 1.450, 2.957]
% For sigma = 0.0035, obj_star = 0.0099, obj_min = 0.011
success_star5 = [65.54, 73.75, 80.56, 86.17, 90.08, 93.24, 95.19, 96.94, 98.25];
success_minus5 = [53.43, 61.86, 71.07, 76.91, 82.23, 86.76, 90.71, 93.50, 95.73];

% Plot success rates for robust-IKs
figure(1)
plot(rad_clearance, success_star1, "bo-", "LineWidth", 1, "MarkerFaceColor", "b");
hold on;
plot(rad_clearance, success_star2, "go-", "LineWidth", 1, "MarkerFaceColor", "g");
plot(rad_clearance, success_star3, "mo-", "LineWidth", 1, "MarkerFaceColor", "m");
plot(rad_clearance, success_star4, "ko-", "LineWidth", 1, "MarkerFaceColor", "k");
plot(rad_clearance, success_star5, "co-", "LineWidth", 1, "MarkerFaceColor", "c");

% plot(rad_clearance, success_star2, "*-", "LineWidth", 1);
% plot(rad_clearance, success_star3, "+-", "LineWidth", 1);
% plot(rad_clearance, success_star4, "x-", "LineWidth", 1);
% plot(rad_clearance, success_star5, "d-", "LineWidth", 1);

xlabel("radial clearance [m]")
ylabel("Success rate %")
title("Success rate of \Theta^* with varying clearance for different poses")
legend("Pose 1", "Pose 2", "Pose 3", "Pose 4", "Pose 5", 'Location', 'SouthEast')
grid on;




