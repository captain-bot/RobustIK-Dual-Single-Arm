function [analytical_jac] = relative_jacobian(gst0_l, gst0_r, gst_l, gst_r, transform_upto_l, transform_upto_r, joint_axes, q_axes, theta)
    % This code computes the spatial and analytical relative Jacobian.
    % Anirban Sinha, State University of New York, Stony Brook
    % Updated: July 31st, 2019

    dim = 3;
    num_of_joints = length(theta);
    spatial_jac = zeros(dim+3, num_of_joints);

    % Fill the first 7 columns of spatial jacobian matrix
    for i = 1:num_of_joints/2
        if i < num_of_joints/2
            g = transform_upto_l(:,:,i+1)\gst_l;
        else
            g = gst0_l;
        end
        Ad_g = get_adjoint(g);
        Ad_g_inv = Ad_g\eye(6, 6);

        omega = joint_axes(:,i);
        q = q_axes(:,i);
        xi = [cross(-omega, q); omega];

        xi_prime = -Ad_g_inv*xi;

        spatial_jac(:,num_of_joints/2 +1 - i) =  xi_prime;
    end

    % Fill the last 7 columns of spatial jacobian matrix
    for j = 1:num_of_joints/2
        if j > 1
            g = gst_l\transform_upto_r(:,:,j+1);
        else
            g = gst_l;
        end        
        Ad_g = get_adjoint(g);
        omega = joint_axes(:,num_of_joints/2 + j);
        q = q_axes(:,num_of_joints/2 + j);
        xi = [cross(-omega, q); omega];
        
        if j > 1
            xi_prime = Ad_g*xi;
        else
            Ad_g_inv = Ad_g\eye(6, 6);
            xi_prime = Ad_g_inv*xi;
        end

        spatial_jac(:,num_of_joints/2 + j) =  xi_prime;
    end

    % Analytical Jacobian
    g_rel = gst_l\gst_r;
    p = g_rel(1:3, 4);
    p_hat = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    analytical_jac = [eye(3,3) -p_hat; zeros(3,3) eye(3,3)]*spatial_jac;
end

function [Adj_mat] = get_adjoint(g)
    R = g(1:3,1:3);
    p = g(1:3,4);
    p_hat = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    temp1 = p_hat*R;
    Adj_mat = [R temp1];
    temp2 = [zeros(3,3) R];
    Adj_mat = [Adj_mat; temp2];
end
