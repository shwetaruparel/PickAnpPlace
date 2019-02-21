# Robotic Arm : Pick and Place Project

[//]: # (Image References)
[image_1]: ./images/JointsLinks.png
[image_2]: ./images/Joints_Links.png
[image_3]: ./images/GenTM.png

--
## Kinematic Analysis
#### Explored Forward Kinematics demo and evaluated the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and to derive its DH parameters.

![Joints and Links][image_1]

![Joints and links from classroom][image_2]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0      | 0     | 0.75 | q1
1->2 | - pi/2 | 0.35  | 0    | q2-pi/2
2->3 | 0      | 1.25  | 0    | q3
3->4 | -pi/2  | -0.054| 1.5  | q4
4->5 | pi/2   | 0     | 0    | q5
5->6 | -pi/2  | 0     | 0    | q6
6->EE| 0      | 0     | 0.303| 0

#### Individual Transform Matrices about each Joint

1. Individual Transform matrix based on modified DH Parameters 
T0_1 :: Matrix([[cos(q1), -sin(q1), 0, 0], 
                [sin(q1), cos(q1), 0, 0], 
                [0, 0, 1, 0.750000000000000], 
                [0, 0, 0, 1]])

T1_2 :: Matrix([[cos(q2 - 0.5*pi), -sin(q2 - 0.5*pi), 0, 0.350000000000000], 
                [0, 0, 1, 0], 
                [-sin(q2 - 0.5*pi), -cos(q2 - 0.5*pi), 0, 0], 
                [0, 0, 0, 1]])

T2_3 :: Matrix([[cos(q3), -sin(q3), 0, 1.25000000000000], 
                [sin(q3), cos(q3), 0, 0], 
                [0, 0, 1, 0], 
                [0, 0, 0, 1]])

T3_4 :: Matrix([[cos(q4), -sin(q4), 0, -0.0540000000000000], 
                [0, 0, 1, 1.50000000000000],
                [-sin(q4), -cos(q4), 0, 0],
                [0, 0, 0, 1]])

T4_5 :: Matrix([[cos(q5), -sin(q5), 0, 0], 
                [0, 0, -1, 0], 
                [sin(q5), cos(q5), 0, 0], 
                [0, 0, 0, 1]])

T5_6 :: Matrix([[cos(q6), -sin(q6), 0, 0],
                [0, 0, 1, 0], 
                [-sin(q6), -cos(q6), 0, 0], 
                [0, 0, 0, 1]])

T6_EE :: Matrix([[1, 0, 0, 0], 
                  [0, 1, 0, 0], 
                  [0, 0, 1, 0.303000000000000], 
                  [0, 0, 0, 1]])
                  
2.Homogeneous transform matrix from base_link to gripper_link using only the position and orientation of the gripper_link
  Generalised Transform Matrix
  ![Joints and links from classroom][image_3]


#### Decouple Inverse Kinematics Problem and derive equations to calculate all individual joint angles.


## Project Implementation




                  
