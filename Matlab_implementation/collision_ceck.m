clc;
clear;
close all;

% Circle radius
circ_rad = 0.1;

% Generate a circle in 2D x,y plane
th = 0:0.02:2*pi;
data.x = circ_rad*cos(th);
data.y = circ_rad*sin(th);
data.z = zeros(size(data.x));

% Generate a rotated circle
rotated_vec = [0.1, 0.1, 0.1];
rotated_vec = rotated_vec/norm(rotated_vec);
fprintf("The rotated vector is: \n"); disp(rotated_vec);

% Compute the rotation matrix between [0, 0, 1] and rotated_vec
rotm = rotmat([0, 0, 1], rotated_vec);
fprintf("The rotation matrix between [0, 0, 1] and rotated_vec\n");
disp(rotm);

% Rotate the points on the generated circle
rotated_circ = rotm*[data.x; data.y; data.z];

% Plot the circle
patch(data.x, data.y, data.z, [0.90, 0.80, 0.80])
hold on
patch(rotated_circ(1, :), rotated_circ(2, :), rotated_circ(3, :), [0.80, 0.90, 0.80])
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
grid on
quiver3(0, 0, 0, 1, 0, 0, 0.05, 'linewidth', 2)
quiver3(0, 0, 0, 0, 1, 0, 0.05, 'linewidth', 2)
quiver3(0, 0, 0, 0, 0, 1, 0.05, 'linewidth', 2)


% Find rotation matrix between [0, 0, 1] and rotated_vec
function [Rot_mat] = rotmat(vec, rotated_vec)
    PP1 = vec;
    PP2 = rotated_vec;

    pr_vec = cross(PP1, PP2);             % vector perp to PP1 and PP2
    pr_vec = pr_vec/norm(pr_vec);         % unit vector perp to PP1 and PP2
    om_hat = [0 -pr_vec(3) pr_vec(2);     % axis of rotation (skew symmetric matrix form)
              pr_vec(3) 0 -pr_vec(1);
              -pr_vec(2) pr_vec(1) 0];

    aval = dot(PP1, PP2);
    bval = norm(PP1)*norm(PP2);

    th = acos(aval/bval);                 % angle of rotation

    Rot_mat = eye(3,3) + sin(th)*om_hat + (1 - cos(th))*(om_hat^2); % rotation matrix
    
end