import numpy as np
import math
import ikModuleLeft


# Compute rotation matrix from a unit quaternion
#  Make sure q = [W,X,Y,Z] or q = [q0, q1, q2, q3] format
def unitquat2rotm(q):
    temp_rotm = np.array([[math.pow(q[0], 2) + math.pow(q[1], 2) - math.pow(q[2], 2) - math.pow(q[3], 2),
                2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[1] * q[3] + 2 * q[0] * q[2]],
                [2 * q[1] * q[2] + 2 * q[0] * q[3],
                math.pow(q[0], 2) - math.pow(q[1], 2) + math.pow(q[2], 2) - math.pow(q[3], 2),
                2 * q[2] * q[3] - 2 * q[0] * q[1]],
                [2 * q[1] * q[3] - 2 * q[0] * q[2], 2 * q[2] * q[3] + 2 * q[0] * q[1],
                math.pow(q[0], 2) - math.pow(q[1], 2) - math.pow(q[2], 2) + math.pow(q[3], 2)]])

    # Norlize columns
    for i in range(3):
        temp_rotm[:, i] /= norm(temp_rotm[:, i])
    return temp_rotm


# Compute norm of a vector
def norm(v):
    return math.sqrt(np.dot(v, v))


# Gripper pose wrt arm_mount frame
P_arm_mount_2_gripper = np.array([0.1933789641, -0.5356598663, 0.1838289519])
Q_arm_mount_2_gripper = np.array([-0.2845541727, 0.2096669616, 0.6764595668, -0.6461200682])

# Get rotation matrix from quaternion
unitQ = Q_arm_mount_2_gripper/norm(Q_arm_mount_2_gripper)
R_arm_mount_2_gripper = unitquat2rotm(unitQ)
# print(R_arm_mount_2_gripper)

# Prepare the input for compIKs method
adjust_length = 0.1077 # for extended narrow gripper. For standard narrow gripper use adjust_length = 0.067
P_ikfast = P_arm_mount_2_gripper - adjust_length*R_arm_mount_2_gripper[:, 2]
print(P_ikfast)
input_pose_matrix = np.hstack((R_arm_mount_2_gripper, P_ikfast.reshape(3, 1)))
ikfast_pose_flattened = input_pose_matrix.reshape(12, ).tolist()
print(ikfast_pose_flattened)
config_num=1
ikModuleLeft.compIKs(config_num, ikfast_pose_flattened)
# Note: IK solutions will be written in file named "ik_sol_config1.txt"

# take one of the solutions in "ik_sol_config1.txt" and check whether you get back the right pose again
js = [0.48053, 0.739716, -1.69314, 2.04778, 0.951574, 0.965272, -0.610338]
fk_sol = ikModuleLeft.compFK(js)
print(fk_sol)


