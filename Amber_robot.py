import socket
from ctypes import *
from scipy.spatial.transform import Rotation

from cmd_py.cmd_04_move_j import Amber_Robot_move_j
from cmd_py.cmd_09_gripper_ctrl import Gripper_Ctrl
from cmd_py.cmd_01_get_status import Get_status

from kinematics.pin_kinematics import Pin_Kinematics

import numpy as np
import math
import csv
import time


class Amber_Robot:

    def __init__(self, IP_ADDR="192.168.50.115", is_use_gripper=False, gripper_ip=""):  # Change the ip address here
        self.IP_ADDR = IP_ADDR
        self.is_use_gripper = is_use_gripper
        if self.is_use_gripper:
            self.gripper_ip = gripper_ip
            self.gripper = Gripper_Ctrl(self.gripper_ip)
        self.amber_robot_move_j = Amber_Robot_move_j(self.IP_ADDR)
        self.pin_kinematics = Pin_Kinematics()
        self.get_status_instance = Get_status(self.IP_ADDR)


    def get_status(self):
        status = self.get_status_instance.get_status()
        return status.pos


    def move_j(self, joint_config):
        self.amber_robot_move_j.move_j(joint_config)
        return True



    def move_j_p(self, tool_configuration):
        result, joint_config = self.pin_kinematics.ik(tool_configuration)
        if result == "success":
            self.move_j(joint_config)
            return True
        else:
            print("ik error")
            return False



    def R2rpy(self, R):
        # assert (isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            r = math.atan2(R[2, 1], R[2, 2])
            p = math.atan2(-R[2, 0], sy)
            y = math.atan2(R[1, 0], R[0, 0])
        else:
            r = math.atan2(-R[1, 2], R[1, 1])
            p = math.atan2(-R[2, 0], sy)
            y = 0
        return np.array([r, p, y])


    def xyz_compensate(self, xyz, rpy, d):

        r = Rotation.from_euler('xyz', rpy, degrees=False)
        rotation_matrix = r.as_matrix()
        new_xyz = xyz - d * rotation_matrix[:, 2]

        return [new_xyz[0], new_xyz[1], new_xyz[2]]

    def grasp_with_gripper2(self, xyz, rpy, d):

        new_xyz = self.xyz_compensate(xyz, rpy, d)
        cartesian_config = new_xyz + rpy

        self.move_j_p(cartesian_config)

        self.gripper_open()

        self.gripper_close()

        flag = True

        if flag == True:
            return True
        else:
            return False



if __name__ == "__main__":
    amber_robot = Amber_Robot()
    # csv_file_path = "depth_camera\test\kinematics\output.csv"
    # amber_robot.move_j_p(csv_file_path)
