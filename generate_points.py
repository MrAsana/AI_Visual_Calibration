#!/usr/bin/env python3

import time 
import numpy as np
import pandas as pd
from kinematics.pin_kinematics import Pin_Kinematics


kinematics = Pin_Kinematics()

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


x_values = np.linspace(Xmin, Xmax, num_points)
y_values = np.linspace(Ymin, Ymax, num_points)
z_values = np.linspace(Zmin, Zmax, num_points)


results = []


counter = 0
qualified_point = 0
# 遍历立方体内的点
for x in x_values:
    for y in y_values:
        for z in z_values:
            SafetyFlag = False
            counter += 1
            print(f"Processing Point #{counter} | {x},{y},{z}")
            time.sleep(0.5)
            

            cartesian_config = [x, y, z, End_Effector_Rotation[0], End_Effector_Rotation[1], End_Effector_Rotation[2]]
            ik_result = kinematics.ik(cartesian_config)

            if ik_result[0] == 'success':

                print(ik_result[0])
                joint_position = ik_result[1]
                fk_result = kinematics.fk(joint_position)

                i = 0
                while i < 6 and abs(cartesian_config[i] - fk_result[i]) < 0.001:
                    i += 1
                for i in range (7):
                    if joint_position[i] > Safety_Limit:
                        SafetyFlag = True
                        print("Rejected due to safety reasons")
                if i >= 6 and not SafetyFlag:
                    results.append([x, y, z] +[End_Effector_Rotation[0],End_Effector_Rotation[1],End_Effector_Rotation[2]]+ list(joint_position))
                    qualified_point+=1
                print(qualified_point)
            else:
                print("This point is not reachable")

df = pd.DataFrame(results, columns=['X', 'Y', 'Z','Roll','Pitch','Yaw', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7'])
df.to_csv('output.csv', index=False)
print("Generated output.csv ")
