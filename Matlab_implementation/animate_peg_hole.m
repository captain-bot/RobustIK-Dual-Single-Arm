function [] = animate_peg_hole(peg_pose, hole_pose, pegd, holed)
    % This function animates peg's motion towards hole direction. It takes
    % four input arguments which is described as following,
    % 1. peg_pose: pose of peg frame wrt base frame
    % 2. hole_pose: pose of hole frame wrt base frame
    % 3. pegd: a structure peg's radius and length
    % 4. holed: a structure hole's radius and length
    % If you need to control peg's linear velocity, you may want to change
    % the value of peg_linear_velocity variable.

    % decide peg's linear velocity
    peg_linear_velocity = 0.003;

    % peg_pose and hole_pose are wrt base frame
    figure('Renderer', 'painters', 'Position', [10 10 1200 900]);

    % Circle radius
    peg_radius = pegd.radius; hole_radius = holed.radius;
    peg_length = pegd.length;  hole_length = holed.length;

    % Generate a unit radius circle in 2D x,y plane
    circ_rad = 1;
    th = 0:0.02:2*pi;
    data.x = circ_rad*cos(th);
    data.y = circ_rad*sin(th);
    data.z = zeros(size(data.x));

    % Generate a vertical peg
    num_pts_circum = 100;
    [peg_cyn.x, peg_cyn.y, peg_cyn.z] = cylinder(peg_radius, num_pts_circum);
    peg_cyn.z = peg_length * peg_cyn.z;

    % Generate a vertical hole
    num_pts_circum = 100;
    [hole_cyn.x, hole_cyn.y, hole_cyn.z] = cylinder(hole_radius, num_pts_circum);
    hole_cyn.z = hole_length*hole_cyn.z;

    % Orientation of base frame
    x_base = [1; 0; 0]; y_base = [0; 1; 0]; z_base = [0; 0; 1];

    % Configuration of hole and peg tip frames wrt base frame
    right_hole_p = hole_pose(1:3, 4);
    right_hole_R = hole_pose(1:3, 1:3);

    left_peg_p = peg_pose(1:3, 4);
    left_peg_R = peg_pose(1:3, 1:3);

    % Rotate and translate the points on the generated circle
    rotated_circ_left = left_peg_p + peg_radius*left_peg_R*[data.x; data.y; data.z];
    rotated_circ_right = right_hole_p + hole_radius*right_hole_R*[data.x; data.y; data.z];

    % Rotate and translate the points on the generated peg
    % get points at the two rings and rotate them separately:
    peg_positionOld1 = [peg_cyn.x(1,:)', peg_cyn.y(1,:)', peg_cyn.z(1,:)'];
    peg_positionOld2 = [peg_cyn.x(2,:)', peg_cyn.y(2,:)', peg_cyn.z(2,:)'];
    peg_positionNew1 = peg_positionOld1*left_peg_R';
    peg_positionNew2 = peg_positionOld2*left_peg_R';

    % Rotate and translate the points on the generated hole
    % get points at the two rings and rotate them separately:
    hole_positionOld1 = [hole_cyn.x(1,:)', hole_cyn.y(1,:)', hole_cyn.z(1,:)'];
    hole_positionOld2 = [hole_cyn.x(2,:)', hole_cyn.y(2,:)', hole_cyn.z(2,:)'];
    hole_positionNew1 = hole_positionOld1*right_hole_R';
    hole_positionNew2 = hole_positionOld2*right_hole_R';

    % reassemble the two sets of points into X Y Z format:
    right_gripper = right_hole_p - hole_length*right_hole_R(:,3);
    hole_new_cyn.x = [hole_positionNew1(:,1) + right_gripper(1), hole_positionNew2(:,1) + right_gripper(1)];
    hole_new_cyn.y = [hole_positionNew1(:,2) + right_gripper(2), hole_positionNew2(:,2) + right_gripper(2)];
    hole_new_cyn.z = [hole_positionNew1(:,3) + right_gripper(3), hole_positionNew2(:,3) + right_gripper(3)];

    % Draw hole circle
    draw_hole_circ = patch(rotated_circ_right(1, :), rotated_circ_right(2, :), rotated_circ_right(3, :), [0.90, 0.90, 0.90]);
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    title("Peg-in-hole assembly simulation")
    axis('equal')
    xlim([0, 0.9])
    ylim([-0.25, 0.25])
    grid on
    view(-131, 11)
    hold on


    % Plot base frame axes
    quiver3(0, 0, 0, x_base(1), x_base(2), x_base(3), 0.05, 'linewidth', 2, 'color', 'r')
    quiver3(0, 0, 0, y_base(1), y_base(2), y_base(3), 0.05, 'linewidth', 2, 'color', 'g')
    quiver3(0, 0, 0, z_base(1), z_base(2), z_base(3), 0.05, 'linewidth', 2, 'color', 'b')

    % Plot axes of hole-frame in right arm
    draw_hole_x = quiver3(right_hole_p(1), right_hole_p(2), right_hole_p(3), right_hole_R(1, 1), right_hole_R(2, 1), right_hole_R(3, 1), 0.05, 'linewidth', 2, 'color', 'r');
    draw_hole_y = quiver3(right_hole_p(1), right_hole_p(2), right_hole_p(3), right_hole_R(1, 2), right_hole_R(2, 2), right_hole_R(3, 2), 0.05, 'linewidth', 2, 'color', 'g');
    draw_hole_z = quiver3(right_hole_p(1), right_hole_p(2), right_hole_p(3), right_hole_R(1, 3), right_hole_R(2, 3), right_hole_R(3, 3), 0.05, 'linewidth', 2, 'color', 'b');

    % Plot hole cylinder
    draw_hole_cyn = surf(hole_new_cyn.x, hole_new_cyn.y, hole_new_cyn.z, "EdgeColor", "none", "FaceAlpha",  0.8, "FaceColor", "interp", "FaceLighting", "gouraud");

    % Copute left grippers position from peg tip position
    left_gripper = left_peg_p - peg_length*left_peg_R(:,3);

    % Start of the animation loop
    num_itr = 0;
    for i = 1:100
        % Plot the circle
        draw_peg_circ = patch(rotated_circ_left(1, :), rotated_circ_left(2, :), rotated_circ_left(3, :), [0.90, 0.90, 0.90]);

        % Plot peg-frame in left arm
        draw_peg_x = quiver3(left_peg_p(1), left_peg_p(2), left_peg_p(3), left_peg_R(1, 1), left_peg_R(2, 1), left_peg_R(3, 1), 0.05, 'linewidth', 2, 'color', 'r');
        draw_peg_y = quiver3(left_peg_p(1), left_peg_p(2), left_peg_p(3), left_peg_R(1, 2), left_peg_R(2, 2), left_peg_R(3, 2), 0.05, 'linewidth', 2, 'color', 'g');
        draw_peg_z = quiver3(left_peg_p(1), left_peg_p(2), left_peg_p(3), left_peg_R(1, 3), left_peg_R(2, 3), left_peg_R(3, 3), 0.05, 'linewidth', 2, 'color', 'b');

        % Plot peg cylinder
        % reassemble the two sets of points into X Y Z format:    
        peg_new_cyn.x = [peg_positionNew1(:,1) + left_gripper(1), peg_positionNew2(:,1) + left_gripper(1)];
        peg_new_cyn.y = [peg_positionNew1(:,2) + left_gripper(2), peg_positionNew2(:,2) + left_gripper(2)];
        peg_new_cyn.z = [peg_positionNew1(:,3) + left_gripper(3), peg_positionNew2(:,3) + left_gripper(3)];
        draw_peg_cyn = surf(peg_new_cyn.x, peg_new_cyn.y, peg_new_cyn.z, "EdgeColor", "none", "FaceAlpha",  1.0, "FaceColor", "interp", "FaceLighting", "gouraud");
    %     draw_peg_cyn = surf(peg_new_cyn.x, peg_new_cyn.y, peg_new_cyn.z, "EdgeColor", "none", "FaceAlpha",  1.0, "FaceColor", "r", "FaceLighting", "gouraud");

        % Text output of number of iterations passed
        num_itr = num_itr + 1;
        itr_plot = text(0.2, 0.2, 0.45, strcat("iteration: ", num2str(num_itr)));

        % Check Euclidean distance between peg and hole frame
        current_dist = norm(left_peg_p - right_hole_p);

        % If Euclidean distance between peg and hole is smaller than a tol
        % value break out from the animation loop
        if current_dist < 2*1e-3
            fprintf("peg frame: x=%2.4f, y=%2.4f, z=%2.4f\n", left_peg_p(1), left_peg_p(2), left_peg_p(3));
            fprintf("Coordinate of peg frame origin in hole frame: \n"); 
            disp(right_hole_R'*(right_hole_p - left_peg_p));
            peg_tip_hole_plane = right_hole_R'*(right_hole_p - left_peg_p);
            break;
        end

        % Update left gripper's frame position
        left_gripper = left_gripper + peg_linear_velocity*left_peg_R(:, 3);

        % Update left gripper peg's frame position
        left_peg_p = left_peg_p + peg_linear_velocity*left_peg_R(:, 3);

        % Update peg's circular face's position
        rotated_circ_left = rotated_circ_left + peg_linear_velocity*left_peg_R(:, 3);

        pause(0.080);

        delete(draw_peg_circ)
        delete(draw_peg_cyn)
        delete(draw_peg_x)
        delete(draw_peg_y)
        delete(draw_peg_z)
        delete(itr_plot)

        %drawnow;
    end
    drawnow;

    % Plot the final view in hole plane
    figure(2)
    patch(hole_radius*cos(th), hole_radius*sin(th), "b")
    hold on
    patch(peg_tip_hole_plane(1) + peg_radius*cos(th), peg_tip_hole_plane(2) + peg_radius*sin(th), "r")
    axis square tight         % set axis to square
    alpha(0.3)                % set all patches transparency to 0.3
    xlabel("x hole")
    ylabel("y hole")
    title("XY plane of hole frame")
    grid on
end