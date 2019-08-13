clc;
clear;
close all;

R = [-1, 0, 0;
     0, 0, -1;
     0, -1, 0];
 
cynd_radius = 0.02; num_pts_circum = 100; cynd_height = 0.05;
[cyn.x, cyn.y, cyn.z] = cylinder(cynd_radius, num_pts_circum);
cyn.z = cynd_height*cyn.z;
 
% set up rotation matrix:
rotationMatrix = R;

% get points at the two rings and rotate them separately:
positionOld1 = [cyn.x(1,:)',cyn.y(1,:)',cyn.z(1,:)'];
positionOld2 = [cyn.x(2,:)',cyn.y(2,:)',cyn.z(2,:)'];
positionNew1 = positionOld1*rotationMatrix;
positionNew2 = positionOld2*rotationMatrix;

% reassemble the two sets of points into X Y Z format:
new_cyn.x = [positionNew1(:,1),positionNew2(:,1)];
new_cyn.y = [positionNew1(:,2),positionNew2(:,2)];
new_cyn.z = [positionNew1(:,3),positionNew2(:,3)];

figure;
surf(new_cyn.x, new_cyn.y, new_cyn.z);