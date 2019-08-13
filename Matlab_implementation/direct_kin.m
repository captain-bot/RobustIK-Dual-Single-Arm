function [gst, transform_upto_joint] = direct_kin(gst0, type_joint, joint_axes, q_axes, theta)
% This code gives the transformation of the tool frame with respect to the
% base frame for any serial chain robot.
% gst0 is the reference configuration of tool frame w.r.t base frame.
% type_joint is a vector of strings encoding the type of the joint as
% prismatic (P) or revolute (R).  
% joint_axes is a matrix where the ith column gives the axis of the ith
% joint in the base frame.
% q_axes gives a choice of the point on the axis to find the unit twist
% associated with the joint i. If the joint is prismatic, we put in a large
% negative number (-555555) to represent that this is not required.
% 

dim = 3;
num_of_joints = length(type_joint);
gst_temp = eye(dim+1,dim+1);
% The transformation upto each joint is stored in the matrix
% transform_upto_joint. It is a three-diemnsional 4 x 4 x n+1 matrix where n 
% is the number of joints. The first 4 x 4 matrix is the identity matrix when 
% the worl frame and the base frame of the manipulator coincide. Otherwise
% it can be used to represent the base frame of the manipulator with respect 
% to a world frame not located at the base. We will use this matrix in the direct
% velocity kinematics problem.
transform_upto_joint = zeros(dim+1, dim+1, num_of_joints+1);
for i = 1:num_of_joints 
    transform_upto_joint(:,:,i) = gst_temp;
    if strcmp(type_joint(i), 'R')
        omega = joint_axes(:,i);
        q = q_axes(:,i);
        xi = [cross(-omega, q); omega];
    end
    if strcmp(type_joint,'P')
        omega = zeros(3,1);
        v = joint_axes(:,i);
        xi = [v; omega];
    end
    gst_joint_i = exp_twist(xi, theta(i));
   % gst_temp = gst_joint_i*gst_temp;
    gst_temp = gst_temp*gst_joint_i;
end
transform_upto_joint(:,:,end) = gst_temp;
gst = gst_temp*gst0;
end
        
    