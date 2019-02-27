#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya
# Modified By : Shweta Ruparel for IK Calculations.
# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


# Define Modified DH Transformation matrix
# The generalised form can be written as below and then each individual transformation matrix can be obtained

def TFMatrix(alpha, a, d , q):
    TFM = Matrix([[ cos(q),               -sin(q),             0,          a],
                  [ sin(q)*cos(alpha),     cos(q)*cos(alpha),  -sin(alpha),-sin(alpha)*d],
                  [ sin(q)*sin(alpha),     cos(q)*sin(alpha),  cos(alpha), cos(alpha)*d ],
                  [0,                      0,                  0,          1            ]])
    return TFM


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols - Create Joint variables , Link Offset variables, Link Length variables and twist angle variables
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint variables
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # Link Offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # Link Length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # Twist Angles

	# Create Modified DH parameters of KUKA KR210 Serial Manipulator based on the URDF File that provides the joint and link imformation
        s = {   alpha0: 0,       a0: 0,      d1: 0.75, q1: q1,
                alpha1: -pi/2.,  a1: 0.35,   d2: 0,    q2: q2-pi/2.,
                alpha2: 0,       a2: 1.25,   d3: 0,    q3: q3,
                alpha3: -pi/2.,  a3: -0.054, d4: 1.5,  q4: q4,
                alpha4:  pi/2.,  a4: 0,      d5: 0,    q5: q5,
                alpha5: -pi/2.,  a5: 0,      d6: 0,    q6: q6,
                alpha6: 0,       a6: 0,      d7: 0.303,q7: 0  }

	#
	#
	# Define Modified DH Transformation matrix. Calculate individual Transformation Matrices.
        T0_1 = TFMatrix(alpha0,a0,d1,q1).subs(s)
        T1_2 = TFMatrix(alpha1,a1,d2,q2).subs(s)
        T2_3 = TFMatrix(alpha2,a2,d3,q3).subs(s)
        T3_4 = TFMatrix(alpha3,a3,d4,q4).subs(s)
        T4_5 = TFMatrix(alpha4,a4,d5,q5).subs(s)
        T5_6 = TFMatrix(alpha5,a5,d6,q6).subs(s)
        T6_EE = TFMatrix(alpha6,a6,d7,q7).subs(s)

	# Define Transformation matrix from base_link to gripper_link by calculating transformation at each link
	T0_2 = T0_1 * T1_2
	T0_3 = T0_2 * T2_3
	T0_4 = T0_3 * T3_4
	T0_5 = T0_4 * T4_5
	T0_6 = T0_5 * T5_6
	T0_EE = T0_6 * T6_EE
        #T0_EE = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE

	# Extract rotation matrices from the transformation matrices
	# Rotation matrix general form.
	# Create variables for roll, pitch and yaw.
        r,p,y = symbols('r p y')
        # Roll
        R_x = Matrix([  [1,     0,      0,      0],
                        [0,     cos(r), -sin(r),0],
                        [0,     sin(r), cos(r), 0],
                        [0,     0,      0,      1]])

        #Pitch
        R_y = Matrix([  [cos(p),        0,sin(p),       0],
                        [0,             1,0,            0],
                        [-sin(p),       0,cos(p),       0],
                        [0,             0,0,            1]])
	#Yaw
        R_z = Matrix([  [cos(y),-sin(y),0,0],
                        [sin(y),cos(y), 0,0],
                        [0,     0,      1,0],
                        [0,     0,      0,1]])

        #Rotaion Matrix for End Effector
        R_EE = R_z * R_y * R_x

        # Correction needed on account of difference in orientation between Urdf and DH convention for the end effector
        R_Corr = R_z.subs({'y':pi}) * R_y.subs({'p':-pi/2})

        ### Rotation Matrix and Transformation Matrix with correction applied.
        T_Total = T0_EE * R_Corr
        R_EE = R_EE * R_Corr


        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Inverse Kinematics Calculation - Position Problem and Orientation Problem
	    ### End Effector Position Matrix from Simulator planned Pose.

	    EE = Matrix([[px],
                         [py],
                         [pz]])

	    ### Substituting the values of Roll , Pitch and Yaw to obtain the rotation Matrix with Correction applied.
            R_EE = R_EE.subs({'r':roll,'p': pitch,'y':yaw})

            Theta1 = 0
            Theta2 = 0
            Theta3 = 0
            Theta4 = 0
            Theta5 = 0
            Theta6 = 0

	    ### Calculate the Wrist Center Postions
            WC = EE - 0.303 * R_EE[:3,2]
            #print(" MY  Wrist Center Matrix is :", WC)
            ### Calculate individual joint angles
	    ### Using trigonometry and geometry find all the joint angles.

            theta1 = atan2(WC[1],WC[0])
	    #### Using Cosine laws for sides , calculating the values of sides and corresponding angles.
            side_a = 1.501
            side_b = sqrt(pow(sqrt(WC[0]*WC[0] + WC[1]*WC[1])-0.35,2) + pow((WC[2] - 0.75),2))
            side_c = 1.25

            angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a)/(2*side_b*side_c))
            angle_b = acos((side_c*side_c + side_a*side_a - side_b*side_b)/(2*side_c*side_a))
            angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c)/(2*side_a*side_b))

            theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75,sqrt(WC[0]*WC[0] + WC[1]*WC[1])-0.35)
            theta3 = pi/2 - (angle_b + 0.036)

	    #### Rotation Matrix for Joint 1 , 2 ,3  can be extracted from Corresponding Transformation Matrix substituted with joint angles. 
            R0_3 = T0_1[0:3,0:3].subs({'q1':theta1}) * T1_2[0:3,0:3].subs({'q2':theta2}) * T2_3[0:3,0:3].subs({'q3':theta3})

            R3_6 = R0_3.inv("LU") * R_EE[0:3,0:3]


            # Euler angles from Rotation Matrix
            theta4 = atan2(R3_6[2,2],-R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])
            ###
            #FK = T_Total.subs({'q1':theta1,'q2':theta2,'q3':theta3,'q4':theta4,'q5':theta5,'q6':theta6})
    	    #your_ee = [FK[0,3],FK[1,3],FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics

            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
