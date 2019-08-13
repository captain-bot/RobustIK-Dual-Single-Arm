clc;
clear all;
close all;

% This code generates plot for success-rates of two IK solutions with
% varying radial clearance between peg and hole objects.

% Data collection type 1 
rad_clearance = 0.008:-0.0005:0.003;
robust_success = [99.33, 98.81, 98.25, 96.94, 95.19, 93.24, 90.08, 86.17, 80.56, 73.75, 65.54];
worst_success = [97.93, 96.89, 95.73, 93.50, 90.71, 86.76, 82.23, 76.91, 71.07 61.86, 53.43];

% Data collection type 2
sig_val = 0.002:0.0005:0.0055;
objective_robust = [0.0056, 0.0071, 0.0085, 0.0099, 0.0113, 0.0127, 0.0142, 0.0156];
objective_worst = [0.0064, 0.0081, 0.0097, 0.011, 0.0129, 0.0146, 0.016, 0.0178];

% Plot success rates
figure(1)
plot(rad_clearance, robust_success, "o-", "LineWidth", 1, "MarkerFaceColor", "b");
hold on;
plot(rad_clearance, worst_success, "o-", "LineWidth", 1, "MarkerFaceColor", "r");
xlabel("radial clearance [m]")
ylabel("Success rate %")
title("Success rate of \Theta^* and \Theta^- with varying clearance")
legend("\Theta^*", "\Theta^-")
grid on;

% Plot change in objective values with varying \sigma
figure(2)
plot(sig_val, objective_robust, "o-", "LineWidth", 1, "MarkerFaceColor", "b");
hold on;
plot(sig_val, objective_worst, "o-", "LineWidth", 1, "MarkerFaceColor", "r");
xlabel("\sigma [radian]")
ylabel("Objective value")
title("Change in Objective value with varying \sigma value")
legend("\Theta^*", "\Theta^-")
grid on;