Calibrate the Intel® RealSense™ D435(i) with the Amber Robot Arm
We will go through three steps to get the relative position of the robot arm base and camera.

Hardware installation
Install the calibration plate to the top of the robotic arm through the quick release interface.

Arrange the camera as shown in the figure, and calibrate the relative position of the plate and the robotic arm.

The camera should be as perpendicular to the robotic arm as possible. In the case of L1, the camera should be about 70cm to 100cm away from the robotic arm.

calibrate-side-mount.42

calibrate-V-mount.43

calibrate-V-mount.44

Calculate calibration points
Open generate_points.py to check user options

The existing csv file is suitable for amber-L1, the rotation direction of [0,1.5707,0] and the above installation method, so for testing you can skip this step.

User options
# User options (change me)
# --------------- Setup options ---------------
# === Workspace ===
Xmin, Xmax = -0.2, 0.2
Ymin, Ymax = -0.2, 0.2
Zmin, Zmax = 0.30, 0.50
# =================
End_Effector_Rotation = [0,1.5707,0]
Safety_Limit = 2
num_points = 5
Workspace that needs to be operated, It is not recommended to set a Zmin value that is too low. Zmin that is too low can easily cause collisions.
Rotation of the end effector during operation, The direction of rotation [0,1.5707,0] applies to the mounting method shown above.
The safety limit on the joint, the default is 2, which means that the point where the joint needs to rotate more than 2 radians will never be generated.
Points obtained in each dimension, if num_point is 5, the program will try 5x5x5=125 points, This only represents the points that will be attempted, not the final points produced. More points will result in higher accuracy, but will result in longer calibration time.
Execute
python generate_points.py
This program will generate output.csv for subsequent use.
Calibration process
Before running, please ensure that the robotic arm is in the initial position, online and enters position mode.

Please ensure that the camera is installed in a suitable and stable position. When the calibration process begins, it is not allowed to change the relative position of the camera and the robot arm base.

Open and change the IP address of your robot.

class Amber_Robot:
    def __init__(self, IP_ADDR="192.168.50.115", is_use_gripper=False, gripper_ip=""):  # Change the ip address here
User options
Open amber_calibrate.py to check user options

checkerboard_offset_from_tool = [0, 0, 0]
You can set the offset of the calibration plate from the end of the robot arm here.

Execute
python amber_calibrate.py
For a 32-point process, it takes about 300-400 seconds

Verification
User options
Open touch.py to check user options

tool_orientation = [0.0, 1.5707, 0.0]
Fill in the rotation direction here, which needs to be consistent with the one filled in in the above process.

Execute
python touch.py
The program will automatically open two windows to obtain a depth image and an RGB image from the camera. The window showing the RGB image can be clicked. When you click on the checkerboard (a random point on checkboard) in the RGB window, the program will control the robot arm to move, trying to align the center of the checkerboard to the click point. Based on the above test process, we can verify the accuracy of the calibration.
