from kinematic_utilities import *
from robot_parameters import *
import matplotlib.pyplot as plt


"""
Given a IK-pair, this piece of code computes objective value (P + \gamma O) for
a particular c value, where c = (k + \sigma)**2.
"""

# User given IK-pair
# Robust IK pair
robust_js_l = [-0.361758, 0.320735, -2.99435, 0.572053, 1.27946, 1.93275, -0.493638]
robust_js_r = [0.493515, 0.550733, 2.88105, 1.20904, -1.36702, 1.55162, 0.838913]
# Worst IK pair
worst_js_l = [-0.120334, 0.0840508, -1.98086, 0.50692, 0.323614, 1.80815, -0.346687]
worst_js_r = [0.278199, -0.710397, 0.709628, 1.20355, -2.08902, -1.33616, 3.04857]

wr_c = np.vstack((wr_l, wr_r))
qr_c = np.vstack((qr_l, qr_r))

err_total_robust, err_total_worst = [], []

# Standard deviation list
sv_list = [0.0015+i*0.0005 for i in range(8)]
k_num = 2

for i in range(2):
    if i == 0:
        js_l, js_r = robust_js_l, robust_js_r
    else:
        js_l, js_r = worst_js_l, worst_js_r

    # Solve forward kinematics for left arm
    gst_l, transform_upto_joint_left = exp_direct_kin(gst0_l, wr_l, qr_l, js_l)

    # Solve forward kinematics for right arm
    gst_r, transform_upto_joint_right = exp_direct_kin(gst0_r, wr_r, qr_r, js_r)

    # For each value of sigma compute error
    for sv in sv_list:
        # Compute desired relative gripper poses
        g_relative_desired = np.dot(inv(gst_l), gst_r)
        pd = g_relative_desired[:3, 3]
        Rd = g_relative_desired[:3, :3]
        qd = rotm2quat(Rd)

        if qd is None:
            print("skipping")
        else:
            theta_c = js_l + js_r
            analytical_relative_jacobian, spatial_relative_jacobian = \
                relative_jac(gst0_l, gst0_r, gst_l, gst_r, transform_upto_joint_left, transform_upto_joint_right, wr_c,
                             qr_c, theta_c)

            # Separate position and orientation Jacobians
            jp = analytical_relative_jacobian[:3, :]
            jr = analytical_relative_jacobian[3:, :]

            # Compute position error bound
            err_p = position_err_bound(jp, (sv*k_num)**2)
            print("\nComputed position error: %2.6f meters\n" % err_p)

            # Compute orientation error bound
            #  Make sure q = [W,X,Y,Z] or q = [q0, q1, q2, q3] format
            err_r, q_star = rotation_err_bound(jr, (sv*k_num)**2, qd)
            print("Computed rotation error: %2.6f radians\n" % err_r)

            # Compute combined position and orientation error for dual-arm assembly
            R_a = unitquat2rotm(q_star)
            R_d = unitquat2rotm(qd)
            err_total = err_p + l_hole * norm(R_a[:3, 2] - R_d[:3, 2])
            print("Combined position and orientation error: %2.6f meters\n" % err_total)

            if i == 0:
                err_total_robust.append(err_total)
            else:
                err_total_worst.append(err_total)

# Plot result
plt.plot(sv_list, err_total_robust, linestyle="--", marker="o", color="b", label=r"$\Theta^*$")
plt.plot(sv_list, err_total_worst, linestyle="-", marker="o", color="r", label=r"$\Theta^-$")
plt.xlabel(r"$\sigma$ [radian]")
plt.ylabel(r"$\mathbb{P} + \gamma \mathbb{O}$ = Objective value [meters]")
plt.title(r"$\sigma$ Vs. Objective-value")
plt.legend()
plt.axis("auto")
plt.grid()
plt.show()
