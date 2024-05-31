#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from amber_realsenseD435 import Camera
from Amber_robot import Amber_Robot
mycamera = Camera()
amber_robot = Amber_Robot() 

# User options (change me)
# --------------- Setup options ---------------

tool_orientation = [0.0, 1.5707, 0.0]
# ---------------------------------------------


# Callback function for clicking on OpenCV window
click_point_pix = ()
camera_color_img, camera_depth_img = mycamera.get_data()
def mouseclick_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global mycamera, kinematics,amber_robot_move_j, click_point_pix
        click_point_pix = (x,y)
        cam_pose,cam_depth_scale = mycamera.get_cailbrated_data()

        # Get click point in camera coordinates
        click_z = camera_depth_img[y][x] * cam_depth_scale
        click_x = np.multiply(x-mycamera.intrinsics[0][2],click_z/mycamera.intrinsics[0][0])
        click_y = np.multiply(y-mycamera.intrinsics[1][2],click_z/mycamera.intrinsics[1][1])
        if click_z == 0:
            return
        click_point = np.asarray([click_x,click_y,click_z])
        click_point.shape = (3,1)

        # Convert camera to robot coordinates
        camera2robot = cam_pose
        target_position = np.dot(camera2robot[0:3,0:3],click_point) + camera2robot[0:3,3:]

        target_position = target_position[0:3,0]
        print(target_position)
        print(target_position.shape)
        
        amber_robot.move_j_p([target_position[0],target_position[1],target_position[2],
                              tool_orientation[0],tool_orientation[1],tool_orientation[2]])




# Show color and depth frames
cv2.namedWindow('color')
cv2.setMouseCallback('color', mouseclick_callback)
cv2.namedWindow('depth')

while True:
    camera_color_img, camera_depth_img = mycamera.get_data()
    bgr_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
    if len(click_point_pix) != 0:
        bgr_data = cv2.circle(bgr_data, click_point_pix, 7, (0,0,255), 2)
    cv2.imshow('color', bgr_data)
    cv2.imshow('depth', camera_depth_img)
    
    if cv2.waitKey(1) == ord('c'):
        break

cv2.destroyAllWindows()
