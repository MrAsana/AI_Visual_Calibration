import socket
from ctypes import *

'''
A simple example for controlling a single joint move once by python

Ref: https://github.com/MrAsana/AMBER_B1_ROS2/wiki/SDK-&-API---UDP-Ethernet-Protocol--for-controlling-&-programing#2-single-joint-move-once
C++ version:  https://github.com/MrAsana/C_Plus_API/tree/master/amber_gui_4_node

'''
                                          # ROS master's IP address




IP_ADDR = "192.168.50.92"                                           # ROS master's IP address

class robot_joint_position(Structure):                              # ctypes struct for send
    _pack_ = 1                                                      # Override Structure align
    _fields_ = [("cmd_no", c_uint16),                               # Ref:https://docs.python.org/3/library/ctypes.html
                ("length", c_uint16),
                ("counter", c_uint32),
                ]

class robot_mode_data(Structure):                                   # ctypes struct for receive
    _pack_ = 1
    _fields_ = [("cmd_no", c_uint16),
                ("length", c_uint16),
                ("counter", c_uint32),
                ("pos", c_float*8),
                ("speed", c_float*8),
                ("Cartesian_pos", c_float*6),
                ("Cartesian_speed", c_float*6),
                ("Arm_Angle", c_float),
                ]

class Get_status:
    def __init__(self, IP_ADDR="192.168.50.207"):
        self.IP_ADDR = IP_ADDR
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 12323))

    def status(self, payloadS):
        payloads = robot_joint_position(1, 8, 11)
        self.socket.sendto(payloadS, (self.IP_ADDR, 25001))
        print("Sending: cmd_no={:d}, "
              "length={:d}, counter={:d},".format(payloadS.cmd_no,
                                                  payloadS.length,
                                                  payloadS.counter))
    
        data, addr = self.socket.recvfrom(1024)
        print("Receiving: ", data.hex())
        payloadR = robot_mode_data.from_buffer_copy(data)
        return payloadR.pos

if __name__ == "__main__":
    get_status = Get_status()






