clc
clear all
close all

rotm = [0.99837616 -0.01578864  0.0548419;
       -0.01376    -0.99911784 -0.03780152;
        0.05544743  0.03691418 -0.997851];
    
q = rotation_mat_quaternion(rotm);
fprintf("Our rotation matrix to quaternion: "); disp(q);

% Use matlab's built-in function to convert rotation matrix
% into quaternion form
fprintf("Matlab's built-in rotm2quat: "); disp(rotm2quat(rotm));

function[quat] = rotation_mat_quaternion(rotm)
    angle = acos((rotm(1, 1) + rotm(2, 2) + rotm(3, 3) - 1)/2);
    temp = sqrt((rotm(3, 2) - rotm(2, 3))^2 + (rotm(1, 3) - rotm(3, 1))^2 + (rotm(2, 1) - rotm(1, 2))^2);
    w(1) = (rotm(3, 2) - rotm(2, 3))/ temp;
    w(2) = (rotm(1, 3) - rotm(3, 1))/ temp;
    w(3) = (rotm(2, 1) - rotm(1, 2))/ temp;
    fprintf("angle: "); disp(angle);
    fprintf("axis: "); disp(w);
    quat = [cos(angle/2), w*sin(angle/2)];
    quat = quat/norm(quat);
    
    % Use matlab's built-in function to convert rotation matrix
    % into axis angle form
    fprintf("Matlab's built-in rotm2axang: "); disp(rotm2axang(rotm));
end