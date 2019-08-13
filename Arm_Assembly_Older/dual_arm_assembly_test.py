from kinematic_utilities import *
from robot_parameters import *

"""
This code does the following:
1. using joint solutions of left and right arms, it computes poses of "left_gripper" and "right_gripper"
2. then it computes "g_relative_desired" and extracts "pd" and "Rd" from that
3. then it computes "analytical_relative_jacobian" (along with "spatial_relative_jacobian") and extracts "jp", "jr"
4. then it computes "err_p" and "err_r" separately
5. finally it computes combined error "err_total" for dual-arm assembly

Note: all the helper function used here can be found in kinematic_utilities.py file
Note: all the robot parameters used here has been taken from robot_parameters.py file
"""

# Load joint solutions
js_l = [-0.350, -0.599, -0.418, 0.891, -1.177, 1.795, 0.375]
js_r = [0.129, -0.790, 0.599, 1.360, 1.160, 1.525, -0.557]

# Transform joint axes and origin locations into base frame for both arms
wr_l = np.dot(T_base2leftarmmount[:3, :3], wr.T).T
qr_l = (np.dot(T_base2leftarmmount[:3, :3], qr.T) + T_base2leftarmmount[:3, 3].reshape(3, 1)).T
gst0_l = np.dot(T_base2leftarmmount, gst0)

wr_r = np.dot(T_base2rightarmmount[:3, :3], wr.T).T
qr_r = (np.dot(T_base2rightarmmount[:3, :3], qr.T) + T_base2rightarmmount[:3, 3].reshape(3, 1)).T
gst0_r = np.dot(T_base2rightarmmount, gst0)

# Solve forward kinematics for left and right arms
gst_l, transform_upto_joint_left = exp_direct_kin(gst0_l, wr_l, qr_l, js_l)
gst_r, transform_upto_joint_right = exp_direct_kin(gst0_r, wr_r, qr_r, js_r)

# Compute relative end-effector poses
g_relative_desired = np.dot(inv(gst_l), gst_r)
pd = g_relative_desired[:3, 3]
Rd = g_relative_desired[:3, :3]
qd = rotm2quat(Rd)
print("\ng_rel (pose of right_gripper wrt left_gripper): ")
print(g_relative_desired)

# Compute Relative-Analytical-Jacobian matrix
wr_c = np.vstack((wr_l, wr_r))
qr_c = np.vstack((qr_l, qr_r))
theta_c = js_l + js_r
analytical_relative_jacobian, spatial_relative_jacobian = \
    relative_jac(gst0_l, gst0_r, gst_l, gst_r, transform_upto_joint_left, transform_upto_joint_right, wr_c, qr_c, theta_c)

# Separate position and orientation Jacobians
jp = analytical_relative_jacobian[:3, :]
jr = analytical_relative_jacobian[3:, :]

# Compute position error
err_p = position_err_bound(jp, c_val)
print("\nComputed position error: %2.6f meters\n" % err_p)

# Compute orientation error
#  Make sure q = [W,X,Y,Z] or q = [q0, q1, q2, q3] format
err_r, q_star = rotation_err_bound(jr, c_val, qd)
print("Computed rotation error: %2.6f radians\n" % err_r)

# Compute combined position and orientation error for dual-arm assembly
R_a = unitquat2rotm(q_star)
R_d = unitquat2rotm(qd)
err_total = err_p + l_hole*norm(R_a[:3, 2] - R_d[:3, 2])
print("Combined position and orientation error: %2.6f meters\n" % err_total)

# print("\nAnalytical relative jacobian")
# print(analytical_relative_jacobian)
#
# print("\nSpatial relative jacobian")
# print(spatial_relative_jacobian)




