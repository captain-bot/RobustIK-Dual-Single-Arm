import ikModuleLeft
import ikModuleRight
import numpy as np
import matplotlib.pyplot as plt


# # Joint solutions
# left_js = [-0.361758, 0.320735, -2.99435, 0.572053, 1.27946, 1.93275, -0.493638]
# right_js = [0.493515, 0.550733, 2.88105, 1.20904, -1.36702, 1.55162, 0.838913]
#
# # Solve forward kinematics (T_ikfast2arm_mount)
# fk_l = ikModuleLeft.compFK(left_js)
# fk_r = ikModuleRight.compFK(right_js)
#
# # Get the transform of the gripper_frame
# T_gripper2armmount_left = np.vstack((fk_l[:3, :3], 0.067*fk_l[:3, 2]))
# T_gripper2armmount_right = np.vstack((fk_r[:3, :3], 0.067*fk_r[:3, 2]))
#
# print(fk_l)
# print(fk_r)

## Old (peg in hole dual arm uncertainty)
# robust_success_rate = np.array([51.50, 58.90, 65.30, 72.20, 77.50, 81.60, 87.40, 89.70, 92.80, 94.10, 96.00, 97.80, 98.30])
# worst_success_rate = np.array([42.70, 51.60, 59.80, 66.20, 71.20, 76.60, 83.70, 86.10, 88.70, 91.70, 94.30, 97.00, 97.50])
# radial_clearance = np.arange(0.0030, 0.0095, 0.0005)

# New (peg in hole dual arm uncertainty)
robust_success_rate = np.array([49.00, 56.01, 64.69, 70.16, 77.09, 81.81, 86.26, 89.91, 92.16, 94.48, 96.15, 97.35, 98.15])
worst_success_rate = np.array([43.11, 50.44, 57.97, 64.95, 71.29, 77.23, 81.31, 86.08, 88.64, 91.73, 93.97, 95.74, 96.97])
radial_clearance = np.arange(0.0030, 0.0095, 0.0005)

# Plot result
plt.plot(radial_clearance, robust_success_rate, linestyle="--", marker="o", color="b", label=r"$\Theta^*$")
plt.plot(radial_clearance, worst_success_rate, linestyle="-", marker="o", color="r", label=r"$\Theta^-$")
plt.xlabel("radial clearance [m]")
plt.ylabel("Success rate [%]")
# plt.title(r"$\sigma$ Vs. Objective-value")
plt.legend()
plt.axis("auto")
plt.grid()
plt.show()


# # New (pre-grasp single arm uncertainty)
# robust_success_rate = np.array([63.9, 71.2, 77.4, 81.2, 84.3, 88.9, 92.2, 95.1, 95.8, 96.6, 98.5, 98.9])
# worst_success_rate = np.array([54.2, 60.9, 66.3, 73.6, 77.0, 81.8, 85.1, 89.3, 90.0, 91.3, 95.5, 96.3])
# radial_clearance = np.arange(0.0035, 0.0095, 0.0005)

# # Plot result
# plt.plot(radial_clearance, robust_success_rate, linestyle="--", marker="o", color="b", label=r"$\Theta^*$")
# plt.plot(radial_clearance, worst_success_rate, linestyle="-", marker="o", color="r", label=r"$\Theta^-$")
# plt.xlabel("radial clearance [m]")
# plt.ylabel("Success rate [%]")
# # plt.title(r"$\sigma$ Vs. Objective-value")
# plt.legend()
# plt.axis("auto")
# plt.grid()
# plt.show()


# Plots sigma vs. objective value for pre-grasp positioning task
sigval = np.arange(0.0015, 0.0055, 0.0005)
num_std = 3
max_eig_robust = 0.5206
max_eig_worst = 0.7510

obj_robust_array = np.sqrt(max_eig_robust) * num_std * sigval
obj_worst_array = np.sqrt(max_eig_worst) * num_std * sigval

plt.plot(sigval, obj_robust_array, linestyle="--", marker="o", color="b", label=r"$\Theta^*$")
plt.plot(sigval, obj_worst_array, linestyle="-", marker="o", color="r", label=r"$\Theta^-$")
plt.xlabel(r"$\sigma$ [rad]")
plt.ylabel("Position error [m]")
plt.legend()
plt.axis("auto")
plt.grid()
plt.show()

