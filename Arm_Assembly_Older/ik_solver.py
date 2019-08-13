import math
import numpy as np
# import ikModule

import ikModuleLeft
import ikModuleRight


left_arm_mount = \
    np.array([[0.7071, -0.7071, 0, 0.025],
        [0.7071, 0.7071, 0, 0.219],
        [0, 0, 1.0, 0.108],
        [0, 0, 0, 1]])

right_arm_mount = \
    np.array([[0.7071, 0.7071, 0, 0.025],
        [-0.7071, 0.7071, 0, -0.219],
        [0, 0, 1.0, 0.108],
        [0, 0, 0, 1]])


class Pose3d(object):
    def __init__(self):
        self._p = list()
        self._q = list()

    def set_position(self, position):
        self._p = position

    def set_rotquat(self, quat):
        self._q = quat


class IkFastSolver(object):
    # p and q has to be in left_gripper frame with respect to base frame
    def __init__(self, p, q, side):
        self._des_config = Pose3d()
        self._des_config.set_position(p)
        self._des_config.set_rotquat(q)
        self._ikfast_extn_len = 0.00
        self._side = side
        if side == "left":
            self.arm_mount = left_arm_mount
        elif side == "right":
            self.arm_mount = right_arm_mount
        else:
            print("side is undeclared")
            return

    # Adjust ee_pose for IKFast input
    def get_iks(self):
        ee_rotm = self.unitquat2rotm(np.array(self._des_config._q))
        ee_position = np.array(self._des_config._p).reshape((3, 1))
        ee_pose = np.vstack((np.hstack((ee_rotm, ee_position)), np.array([0, 0, 0, 1]).reshape(1, 4)))
        ikfast_pose = np.dot(np.linalg.inv(self.arm_mount), ee_pose)
        ikfast_pose[:3, 3] = ikfast_pose[:3, 3] - self._ikfast_extn_len*ikfast_pose[:3, 2]
        print("IK-fast pose: "), print(ikfast_pose)
        ikfast_pose_flattened = ikfast_pose[:3, :4].reshape(12, ).tolist()
        return self.solve_ik(ikfast_pose_flattened, config_num=1)

    # Compute rotation matrix from a unit quaternion
    #  Make sure q = [W,X,Y,Z] or q = [q0, q1, q2, q3] format
    def unitquat2rotm(self, q):
        temp_rotm = np.array([[math.pow(q[0], 2) + math.pow(q[1], 2) - math.pow(q[2], 2) - math.pow(q[3], 2),
                            2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[1] * q[3] + 2 * q[0] * q[2]],
                            [2 * q[1] * q[2] + 2 * q[0] * q[3],
                            math.pow(q[0], 2) - math.pow(q[1], 2) + math.pow(q[2], 2) - math.pow(q[3], 2),
                            2 * q[2] * q[3] - 2 * q[0] * q[1]],
                            [2 * q[1] * q[3] - 2 * q[0] * q[2], 2 * q[2] * q[3] + 2 * q[0] * q[1],
                            math.pow(q[0], 2) - math.pow(q[1], 2) - math.pow(q[2], 2) + math.pow(q[3], 2)]])
        # Norlize columns
        for i in range(3):
            temp_rotm[:, i] /= self.norm(temp_rotm[:, i])
        return temp_rotm

    # Solve for multiple IKs
    def solve_ik(self, ikfast_pose_flattened, config_num=1):
        if self._side == "left":
            return ikModuleLeft.compIKs(config_num, ikfast_pose_flattened)
        else:
            config_num = 2
            return ikModuleRight.compIKs(config_num, ikfast_pose_flattened)

    # Solve forward kinematics
    def solve_fk(self, js):
        print("Solving forward kinematics")
        # fk_sol = ikModule.compFK(js)
        if self._side == "left":
            fk_sol = ikModuleLeft.compFK(js)
        else:
            fk_sol = ikModuleRight.compFK(js)
        if fk_sol is not None:
            print("Forward kinematics solved("+self._side+"-arm):")
            print(fk_sol)
            return fk_sol
        else:
            return None

    # Compute norm of a vector
    @staticmethod
    def norm(v):
        return math.sqrt(np.dot(v, v))


if __name__ == "__main__":
    # <side>_gripper configuration
    # Make sure q = [W,X,Y,Z] or q = [q0, q1, q2, q3] format

    # Test case for left arm
    # assembly example left arm
    pd_l = [0.635, -0.035, 0.414]         # desired left end-effector position
    qd_l = [0.520, 0.514, -0.510, 0.453]  # desired left end-effector orientation

    # Compute all IK for left-arm
    ik_obj_l = IkFastSolver(pd_l, qd_l, side="left")
    ik_obj_l.get_iks()

    # Take one solution from ik_sol_config1.txt file
    js_l = [0.0178996, -0.355206, -1.38141, 1.66078, -0.462875, 0.855331, -1.40905]

    # Solve forward kinematics wrt base frame
    fk_arm_mount = ik_obj_l.solve_fk(js_l)
    fk_arm_mount = np.vstack((fk_arm_mount, np.array([0, 0, 0, 1])))
    fk_base = np.dot(left_arm_mount, fk_arm_mount)
    print("\nCheck whether you get back same end-effector (left) configuration")
    print(fk_base)

"""
    # Uncomment to test right arm solutions
    # Test case for right arm
    # assembly example right arm
    pd_r = [0.639, -0.008, 0.425]           # desired right end-effector position
    qd_r = [0.510, -0.516, -0.499, -0.474]  # desired right end-effector orientation

    # Compute all IK for right-arm
    ik_obj_r = IkFastSolver(pd_r, qd_r, side="right")
    ik_obj_r.get_iks()

    # Take one solution from ik_sol_config2.txt file
    js_r = [0.270379, 0.783368, 2.47825, 1.78482, 1.87164, -1.27019, -0.687569]

    # Solve forward kinematics wrt base frame
    fk_arm_mount = ik_obj_r.solve_fk(js_r)
    fk_arm_mount = np.vstack((fk_arm_mount, np.array([0, 0, 0, 1])))
    fk_base = np.dot(right_arm_mount, fk_arm_mount)
    print("\nCheck whether you get back same end-effector (right) configuration")
    print(fk_base)
    ik_obj_r.unitquat2rotm(qd_r)
"""
