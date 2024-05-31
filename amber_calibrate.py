#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
import csv
# from UR_Robot import UR_Robot

from Amber_robot import Amber_Robot
from amber_realsenseD435 import Camera
from scipy import optimize
from mpl_toolkits.mplot3d import Axes3D

# User options (change me)
# --------------- Setup options ---------------


amber_robot = Amber_Robot()  # (change me)

# camera
mycamera = Camera()



checkerboard_offset_from_tool = [0, 0, 0]


measured_pts = []
observed_pts = []
observed_pix = []

# Move robot to home pose
print('Connecting to robot...')

## change 3
amber_robot.move_j([0, 0, 0, 0, 0, 0, 0])

# Move robot to each calibration point in workspace
print('Collecting data...')
iteration = 0
time_start_all = time.perf_counter()
with open('./output.csv', 'r') as file:


	iteration += 1


	reader = csv.reader(file)
	next(reader)  # skip first row
	counter = 0 
	good_point = 0
	for row in reader:
		counter+=1
		print (f"========== Moving to point #{counter} ==========")
		time_start = time.perf_counter()
		tool_position =  [float(value) for value in row[0:3]]
		#tool_config = [tool_position[0], tool_position[1], tool_position[2],
		#	   tool_orientation[0], tool_orientation[1], tool_orientation[2]]

		
		joint_values = [float(value) for value in row[6:13]]
		#rotate_direction = [1.0,-1.0,1.0,-1.0,1.0,-1.0,1.0]
		#for i in range(7):
		#	joint_values[i] = joint_values[i]*rotate_direction[i]
		print(joint_values)
		amber_robot.move_j(joint_values)
		# get joint val
		current_joint_values = amber_robot.get_status()

		success = [0,0,0,0,0,0,0]
		while(1):
			current_joint_values = amber_robot.get_status()
			for i in range(len(joint_values)):
				
				if abs(current_joint_values[i] - joint_values[i]) <= 0.01:
					success[i] = 1
			sucsum = 0
			for i in range(len(joint_values)):
				sucsum += success[i]

			if sucsum >= len(joint_values):	
				break
			time.sleep(0.2)
			
		if success:
			print("Success")
		else:
			print("Failed")
		# Waiting for stable
		time.sleep(1)

		# Find checkerboard center
		checkerboard_size = (5, 5)
		refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
		camera_color_img, camera_depth_img = mycamera.get_data()
		bgr_color_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
		gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_RGB2GRAY)
		checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None,
																cv2.CALIB_CB_ADAPTIVE_THRESH)
		print("=== Checkerboard Found ? ===")
		print(checkerboard_found)
		if checkerboard_found:
			corners_refined = cv2.cornerSubPix(gray_data, corners, (5, 5), (-1, -1), refine_criteria)
	
			# Get observed checkerboard center 3D point in camera space
			checkerboard_pix = np.round(corners_refined[12, 0, :]).astype(int)
			checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
			checkerboard_x = np.multiply(checkerboard_pix[0] - mycamera.intrinsics[0][2],
										 checkerboard_z / mycamera.intrinsics[0][0])
			checkerboard_y = np.multiply(checkerboard_pix[1] - mycamera.intrinsics[1][2],
										 checkerboard_z / mycamera.intrinsics[1][1])
			if checkerboard_z == 0:
				cv2.imshow('DepthE', camera_depth_img)
				cv2.waitKey(1000)
				print("?????????????????????????")
				continue

	
			# Save calibration point and observed checkerboard center
			observed_pts.append([checkerboard_x, checkerboard_y, checkerboard_z])
			
			# tool_position[2] += checkerboard_offset_from_tool
			
			
			for i in range (3):
				tool_position[i] = tool_position[i] + checkerboard_offset_from_tool[i]
	
			measured_pts.append(tool_position)
			print (f"measured   = {tool_position}")
			print (f"observed   = {[checkerboard_x, checkerboard_y, checkerboard_z]}")
			print (f"observedpix= {checkerboard_pix}")
			good_point+=1
			print (f"Found {good_point} points")
			
			observed_pix.append(checkerboard_pix)
	
			# Draw and display the corners
			# vis = cv2.drawChessboardCorners(robot.camera.color_data, checkerboard_size, corners_refined, checkerboard_found)
			vis = cv2.drawChessboardCorners(bgr_color_data, (1, 1), corners_refined[12, :, :], checkerboard_found)
			time.sleep(0.5)
			cv2.imwrite('%06d.png' % len(measured_pts), vis)
			cv2.imshow('Calibration', vis)
			time.sleep(5)
			cv2.waitKey(1000)
		time_end = time.perf_counter()
		time_consumed = time_end - time_start
		print(f"Using {time_consumed} seconds for latest point")
print(f"Using {time.perf_counter() - time_start_all} seconds for all")
# Move robot back to home pose
# robot.go_home()

measured_pts = np.asarray(measured_pts)
observed_pts = np.asarray(observed_pts)
observed_pix = np.asarray(observed_pix)
world2camera = np.eye(4)


# Estimate rigid transform with SVD (from Nghia Ho)
def get_rigid_transform(A, B):
	assert len(A) == len(B)
	N = A.shape[0]  # Total points
	centroid_A = np.mean(A, axis=0)
	centroid_B = np.mean(B, axis=0)
	AA = A - np.tile(centroid_A, (N, 1))  # Centre the points
	BB = B - np.tile(centroid_B, (N, 1))
	H = np.dot(np.transpose(AA), BB)  # Dot is matrix multiplication for array
	U, S, Vt = np.linalg.svd(H)
	R = np.dot(Vt.T, U.T)
	if np.linalg.det(R) < 0:  # Special reflection case
		Vt[2, :] *= -1
		R = np.dot(Vt.T, U.T)
	t = np.dot(-R, centroid_A.T) + centroid_B.T
	return R, t


def get_rigid_transform_error(z_scale):
	global measured_pts, observed_pts, observed_pix, world2camera, camera

	# Apply z offset and compute new observed points using camera intrinsics
	observed_z = observed_pts[:, 2:] * z_scale
	observed_x = np.multiply(observed_pix[:, [0]] - mycamera.intrinsics[0][2], observed_z / mycamera.intrinsics[0][0])
	observed_y = np.multiply(observed_pix[:, [1]] - mycamera.intrinsics[1][2], observed_z / mycamera.intrinsics[1][1])
	new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)

	# Estimate rigid transform between measured points and new observed points
	R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
	t.shape = (3, 1)
	world2camera = np.concatenate((np.concatenate((R, t), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

	# Compute rigid transform error
	registered_pts = np.dot(R, np.transpose(measured_pts)) + np.tile(t, (1, measured_pts.shape[0]))
	error = np.transpose(registered_pts) - new_observed_pts
	error = np.sum(np.multiply(error, error))
	rmse = np.sqrt(error / measured_pts.shape[0])
	return rmse


# Optimize z scale w.r.t. rigid transform error
print('Calibrating...')
z_scale_init = 1
optim_result = optimize.minimize(get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
camera_depth_offset = optim_result.x

# Save camera optimized offset and camera pose
print('Saving...')
np.savetxt('camera_depth_scale.txt', camera_depth_offset, delimiter=' ')
get_rigid_transform_error(camera_depth_offset)
camera_pose = np.linalg.inv(world2camera)
np.savetxt('camera_pose.txt', camera_pose, delimiter=' ')
print('Done.')

# DEBUG CODE -----------------------------------------------------------------------------------

# np.savetxt('measured_pts.txt', np.asarray(measured_pts), delimiter=' ')
# np.savetxt('observed_pts.txt', np.asarray(observed_pts), delimiter=' ')
# np.savetxt('observed_pix.txt', np.asarray(observed_pix), delimiter=' ')
# measured_pts = np.loadtxt('measured_pts.txt', delimiter=' ')
# observed_pts = np.loadtxt('observed_pts.txt', delimiter=' ')
# observed_pix = np.loadtxt('observed_pix.txt', delimiter=' ')

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(measured_pts[:,0],measured_pts[:,1],measured_pts[:,2], c='blue')

# print(camera_depth_offset)
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(observed_pts)) + np.tile(camera2robot[0:3,3:],(1,observed_pts.shape[0])))

# ax.scatter(t_observed_pts[:,0],t_observed_pts[:,1],t_observed_pts[:,2], c='red')

# new_observed_pts = observed_pts.copy()
# new_observed_pts[:,2] = new_observed_pts[:,2] * camera_depth_offset[0]
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_new_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(new_observed_pts)) + np.tile(camera2robot[0:3,3:],(1,new_observed_pts.shape[0])))

# ax.scatter(t_new_observed_pts[:,0],t_new_observed_pts[:,1],t_new_observed_pts[:,2], c='green')

# plt.show()
