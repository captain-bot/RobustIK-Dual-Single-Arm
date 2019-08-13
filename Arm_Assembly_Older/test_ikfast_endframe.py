import numpy as np
import math
import ikModuleLeft
import ikModuleRight

"""
This code computes error between end-effector poses obtained from RViz and IKFast forward kinematics
method independently. Since IKFast produces end-effector pose with respect to left_arm_mount frame,
the pd and qd are also taken by considering that. In other words pd and qd are position and orientation
of "left_gripper" frame with respect to "left_arm_mount" frame.

Note: We did not check orientation error because we found that orientations from these two different
sources matches perfectly. That we we are not using qd anywhere but kept it along for completeness.

Author: Anirban Sinha
Date: June 26th, 2019
Contact: anirban.sinha@stonybrook.edu 
"""

###########################################
# Test case 1
###########################################
# Note: standard narrow gripper is used
pd = np.array([0.222, -0.507, 0.182])
qd = [-0.282, 0.209, 0.679, -0.644]
js = [0.3490, -0.2883, -1.1873, 2.1067, 2.4960, -0.7351, -2.5284]

fk_sol = ikModuleLeft.compFK(js)
pa = fk_sol[:, 3]
error = pa - pd
abs_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
print("Absolute error %2.6f" % abs_error)
print("============================================\n")

###########################################
# Test case 2
###########################################
# Note: standard narrow gripper is used
pd = np.array([0.335, -0.567, 0.180])
qd = [-0.308, 0.281, 0.650, -0.636]
js = [-0.1154, -0.5273, -0.6580, 1.4783, -0.9169, 1.4492, 0.6504]

fk_sol = ikModuleLeft.compFK(js)
pa = fk_sol[:, 3]
error = pa - pd
abs_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
print("Absolute error %2.6f" % abs_error)
print("============================================\n")

###########################################
# Test case 3
###########################################
# Note: standard narrow gripper is used
pd = np.array([0.348, -0.614, 0.408])
qd = [-0.317, 0.233, 0.675, -0.624]
js = [-0.0610, -0.8360, -0.8682, 1.3763, -1.0312, 1.4243, 0.1469]

fk_sol = ikModuleLeft.compFK(js)
pa = fk_sol[:, 3]
error = pa - pd
abs_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
print("Absolute error %2.6f" % abs_error)
print("============================================\n")

###########################################################
# Test case right arm with standard narrow gripper
###########################################################
pd_r = np.array([0.350, 0.584, 0.193])
qd_r = np.array([-0.239, -0.355, 0.576, 0.697])
js = [0.630801096432741, -0.602499540215156, -0.105711918042191, 1.400339041322102, 1.463158873826231, 1.910764999999990, -1.036010254667002]
# js = [0.2001, -0.4832, 0.5717, 1.3610, 1.0143, 1.5708, -0.9092]

fk_sol = ikModuleRight.compFK(js)
pa = fk_sol[:, 3]
error = pa - pd_r
abs_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
print("Absolute error %2.6f" % abs_error)
print("============================================\n")

###########################################
# Test case 4
###########################################
pd = np.array([0.3866647738, -0.7262986804, 0.2122544445])
qd = [-0.3063825941, 0.3034236009, 0.6363355209, -0.6396412505]
js = [-0.3976845193, -0.1514806028, -0.8233641879, 0.5361262854, -0.7179030087, 1.6682041068, 0.2143738151]

fk_sol = ikModuleLeft.compFK(js)
pa = fk_sol[:, 3]
error = pa - pd
abs_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
print("Absolute error %2.6f" % abs_error)
print("============================================\n")

###########################################
# Test case 5
###########################################
pd = np.array([0.2753064953, -0.6086908583, 0.4625139831])
qd = [-0.3119848905, 0.3401804399, 0.6083147503, -0.6456747331]
js = [-0.0832184577, -0.8597962316, -0.8881748762, 1.3169225064, -1.0542282965, 1.5780827355, 0.2684466379]

fk_sol = ikModuleLeft.compFK(js)
pa = fk_sol[:, 3]
error = pa - pd
abs_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
print("Absolute error %2.6f" % abs_error)
print("============================================\n")

###########################################
# Test case 6
###########################################
pd = np.array([0.1675057911, -0.787777989, 0.4823510298])
qd = [-0.3059518935, 0.2920510557, 0.625947232, -0.6552020166]
js = [-0.36010199,-0.7727428219,-1.0028399401,1.2179807456,-1.0185632432,1.1711943316,0.1530145836]

fk_sol = ikModuleLeft.compFK(js)
pa = fk_sol[:, 3]
error = pa - pd
abs_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
print("Absolute error %2.6f" % abs_error)
print("============================================\n")

###########################################
# Test case 7
###########################################
pd = np.array([0.5266277956, -0.8345378533, 0.2747493178])
qd = [-0.0957988142, 0.1866472442, 0.6945966688, -0.6881285208]
js = [-0.3156165471, -0.2695971235, -0.9679418772, 0.6895243642, -0.6734175659, 1.0964127681, 0.2649951811]

fk_sol = ikModuleLeft.compFK(js)
pa = fk_sol[:, 3]
error = pa - pd
abs_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
print("Absolute error %2.6f" % abs_error)
print("============================================\n")

###########################################
# Test case 8
###########################################
pd = np.array([0.2962821457, -0.5846495055, 0.3623362943])
qd = [-0.2703177282, 0.3276613036, 0.6411000211, -0.6391847611]
js = [0.1369077853, -0.6496408637, -1.1458836486, 1.4829759267, -0.7443641773, 1.3449176558, 0.3512816004]

fk_sol = ikModuleLeft.compFK(js)
pa = fk_sol[:, 3]
error = pa - pd
abs_error = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
print("Absolute error %2.6f" % abs_error)
print("============================================\n")
