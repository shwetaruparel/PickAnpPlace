# Robotic Arm : Pick and Place Project

[//]: # (Image References)
[image_1]: ./images/JointsLinks.png
[image_2]: ./images/Joints_Links.png
[image_3]: ./images/transmatrice.png
[image_4]: ./images/homo-xform-2.png
[image_5]: ./images/WristCenter.png
[image_6]: ./images/theta2-3.png


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
                  
2. Homogeneous transform matrix from base_link to gripper_link using only the position and orientation of the gripper_link
    Generalised Transform Matrix
  ![Generalised Xform][image_3]
  ![Homo Xform][image_4]
  
  Position and orientation of the gripper link can be calulated from simulation in ROS. Since these values are returned in quaternions,   we can use the transformations.py module from the TF package and use euler_from_quaternions() method to get the roll, pitch, and yaw   values.Position Px, Py, Pz can be calculated from the requested pose.  Wrist Center can be calculated using the following .
  
  ![Wrist Center][image_5]
  
  Where,

  Px, Py, Pz = end-effector positions

  Wx, Wy, Wz = wrist positions

  d6 = from DH table

  _l_ = end-effector length

  Homogenous Rotaion Matrix are defined as

  R_x = Matrix([    [1,     0,      0,      0],
                    [0,     cos(r), -sin(r),0],
                    [0,     sin(r), cos(r), 0],
                    [0,     0,      0,      1]])
  
  r is the value of roll
   
  R_y = Matrix([   [cos(p),        0,sin(p),       0],
                    [0,             1,0,            0],
                    [-sin(p),       0,cos(p),       0],
                    [0,             0,0,            1]])

  
  p is the vale of pitch
  
  R_z = Matrix([  [cos(y),-sin(y),0,0],
                    [sin(y),cos(y), 0,0],
                    [0,     0,      1,0],
                    [0,     0,      0,1]])

  y is the value of yaw.
   
  Rotaion Matrix for End Effector
  
  **R_EE = R_z * R_y * R_x

  **R_EE = R_EE.subs({'r' : roll,'p' : pitch,'y' : yaw})

  Since URDF model does not follow DH Convention. We will need correctional Matrix as follows

  **R_Corr = R_z.subs(y,pi)*R_y.subs(p,-pi/2)
  
  Also, Total Homogeneous transform between base_link and Gripper link with correvtion would be
  **T_Total= T6_EE * R_Corr


#### Decouple Inverse Kinematics Problem and derive equations to calculate all individual joint angles.

Inverse kinematics (IK) is mainly the opposite idea of forwards kinematics. In this case,calculate the joint angles of the manipulatorare calculated based on the pose (i.e., position and orientation) of the end effector.

Research has shown that if either of the following two conditions are satisfied, then the serial manipulator is solvable in closed-form.

1. Three neighboring joint axes intersect at a single point, or

2. Three neighboring joint axes are parallel (which is technically a special case of 1, since parallel lines intersect at infinity)

Kuka KR210 ,six DoF serial manipulator used in the project,has the last three joints to be revolute joints and that satisfy condition 1, such a design is called a spherical wrist and the common point of intersection is called the wrist center. The advantage of such a design is that it kinematically decouples the position and orientation of the end effector. 

It would use the first three joints to control the position of the wrist center while the last three joints would orient the end effector as needed.

We have already seen how to calculate the wrist center positions Wx, Wy, Wz now we need to calculate the values of joint angles.

Once the first three joint variables are known, we can calculate the homogenous transform upto the wrist center **0R3 using T0_1 *T1_2*T2_3***

Final three Joints(4,5,6) can be calculated by considering the following:

Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation by:

R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6

Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of individual rotations between respective links, following holds true:

R0_6 = Rrpy

where,

Rrpy = Homogeneous RPY rotation between base_link and gripper_link as calculated above.

We can substitute the values we calculated for joints 1 to 3 in their respective individual rotation matrices and pre-multiply both sides of the above equation by inv(R0_3) which leads to:

R3_6 = inv(R0_3) * Rrpy

if we look top down view of robotic arm,**Theta 1 = atan2(Wy,Wx)**
Considering a triangle formed between **Joint 2 , Joint 3 and Wrist center** as shown in the figure we can find **theta 2 and theta 3**.

![theta2-3][image_6]



## Project Implementation




                  
