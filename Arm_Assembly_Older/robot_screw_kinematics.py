from kinematic_utilities import *


# Joint angles
# th_ang = [0.1, -0.5, 0.1, 0.1, 0.1, 0.5, 0.1]
# th_ang_left = [-1.27761, 0.627382, 2.15623, 1.34847, 0.153395, -0.881736, 0.528056]
# th_ang_left = [-1.25101, 0.700638, 2.24862, 1.41107, 0.112566, -0.925713, 0.457238]
# th_ang_left = [0.1, -0.5, 0.1, 0.1, 0.1, 0.5, 0.1]
# th_ang_left = [0.2, -0.5, 0.2, 0.5, 0.2, 0.5, 0.2]
# th_ang_left = [0.5, -0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
# th_ang_left = [-1.42575,0.18041,1.61075,1.36355,0.401884,-0.801113,1.0026]
# th_ang_left = [-1.4245, 0.204552, 1.63517, 1.36372, 0.389788, -0.808442, 0.978751]
# th_ang_left = [-1.42575, 0.18041, 1.61075, 1.36355, 0.401884, -0.801113, 1.0026]
# th_ang_left = [0.0142832, 0.153928, -1.60412, 1.41151, 2.65139, 0.730732, 1.95499]
# th_ang_left = [-1.38911, 0.347695, 1.78937, 1.32586, 0.317378, -0.823101, 0.828608]
# th_ang_left = [-1.39439, 0.319398, 1.75762, 1.32478, 0.333024, -0.815772, 0.85719]
# th_ang_left = [0.0228785, 0.175279, -1.60535, 1.38475, -0.487435, -0.698501, -1.16222]
# th_ang_left = [0]*7
# th_ang_left = [-1.40991, 0.220676, 1.67385, 1.33004, 0.400178, -0.801113, 0.928088]
# th_ang_left = [-1.37402, 0.224667, 1.67531, 1.2434, 0.40782, -0.749807, 0.909718]
# th_ang_left = [-1.39634, 0.165701, 1.61067, 1.28665, 0.434171, -0.757137, 0.977453]
# th_ang_left = [-1.40094, -0.162133, 1.28685, 1.37867, 0.568876, -0.683843, 1.3216]
# th_ang_left = [0.0110131, 0.237687, -1.6882, 1.38406, -0.47567, -0.735148, -1.07578]
# th_ang_left = [-1.43166, 0.132954, 1.58806, 1.3814, 0.437099, -0.808442, 1.02531]
# th_ang_left = [-1.40591, 0.210573, 1.6634, 1.32885, 0.403147, -0.801113, 0.941202]
# th_ang_left = [-1.36697,-0.00210062,1.42044,1.24482,0.53068,-0.676513,1.14049]
# th_ang_left = [-1.41842, 0.249569, 1.70472, 1.37063, 0.378151, -0.83776, 0.910214]

# th_ang_left = [-0.282344, -0.979556, -0.503889, 1.7143, 1.9092, -1.48275, 2.14238]
# th_ang_right = [0.131496, 0.766848, .53245, 1.725, 1.99928, -1.43144, -0.739971]
# th_ang_right = [-0.0712194, 0.583393, 2.21057, 1.70046, 2.24055, -1.24088, -0.863624]
# th_ang_right = [0.259358, -1.01575, 0.425209, 1.73519, -1.87658, -1.55604, -2.1768]

th_ang_left = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
th_ang_right = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

# print("\nCurrent joint configuration of Baxter robot:")
# print(th_ang)
# print("=================================\n")

# Link lengths
L0, L1, L2, L3 = 0.27035, 0.36435, 0.37429, 0.22952
o1, o2, o3 = 0.069, 0.069, 0.010
gripper_link = 0.025 + 0.1372

# Base transformation
base_mat_left = np.array([[0.7071, -0.7071, 0.0, 0.0648],
                     [0.7071, 0.7071, 0.0, 0.2584],
                     [0.0, 0.0, 1.0, 0.1190],
                     [0, 0, 0, 1.0]])

base_mat_right = np.array([[0.7071, 0.7071, 0.0, 0.0648],
                     [-0.7071, 0.7071, 0.0, -0.2584],
                     [0.0, 0.0, 1.0, 0.1190],
                     [0, 0, 0, 1.0]])

# Define axis of rotations
w1, w2, w3 = [0, 0, 1], [0, 1, 0], [1, 0, 0]
w4 = w6 = w2
w5 = w7 = w3
wr = np.array([w1, w2, w3, w4, w5, w6, w7])

# Frame origins
q1, q2, q4, q6 = [0, 0, 0], [o1, 0, L0], [o1+L1, 0, L0-o2], [o1+L1+L2, 0, L0-o2-o3]
q3, q5, q7 = q2, q4, q6
qr = np.array([q1, q2, q3, q4, q5, q6, q7])

# Define g_st0
g_st0_left = g_st0_right = np.array([[0, 0, 1, 1.194935],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [-1.0, 0.0, 0.0, 0.19135],
                                    [0.0, 0.0, 0.0, 1.0]])

# Solve direct position kinematics
gst, transform_upto_joint_left = exp_direct_kin(g_st0_left, wr, qr, th_ang_left)
g_ee = np.dot(base_mat_left, gst)
print("baseTend_effector (Left-Arm):")
print(g_ee)
print("=================================\n")

# Solve direct position kinematics
gst_right, transform_upto_joint_right = exp_direct_kin(g_st0_right, wr, qr, th_ang_right)
g_ee_right = np.dot(base_mat_right, gst_right)
print("baseTend_effector (Right-Arm):")
print(g_ee_right)
print("=================================\n")

# Compute spatial jacobian
spatial_jac = velocity_direct_kin(g_st0_left, wr, qr, th_ang_left)
print("Spatial Jacobian of Baxter Robot(Left-Arm):")
print(spatial_jac)
print("=================================\n")

# Transform a unit quaternion in to a rotation matrix
print("Quaternion to Rotation-Matrix")
quat = [0.510, -0.516, -0.499, -0.474]
print("quat: {}".format(quat))
rot_mat = unitquat2rotm(quat)
print("rotation-matrix: {}".format(rot_mat))
