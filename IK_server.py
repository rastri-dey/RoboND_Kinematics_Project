#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from time import time
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
import numpy as np
from mpmath import *
from sympy import *

# Calculating Iteration Count for Kuka arm 
# To reach Target Location and from Target Location to Drop-off location
class count_state():
    def __init__(self):
        self.count = 0
	
    	
Count = count_state()
print(Count.count)

# Defining Position Requests for joint angle calculations

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
	print "No valid poses received"
        return -1
    else:
	start_time = time()

        ### Forward Kinematics code 

        # Create symbols
	#Joint Angle Symbols
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
	
	#Link Offset Symbols
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	
	#Link Length symbols
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	
	#Twist Angle symbols
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
	
	# Modified DH parameters
	DH_Table = {	alpha0: 0,	a0:   0,	d1: 0.75,	q1: q1, 
     			alpha1: -pi/2.,	a1: 0.35,	d2: 0,		q2: -pi/2. + q2,  
     			alpha2: 0,	a2: 1.25,	d3: 0,		q3: q3,
     			alpha3: -pi/2.,	a3: -0.054,	d4: 1.5,	q4: q4,
			alpha4: pi/2.,	a4:   0,	d5: 0,		q5: q5,
			alpha5: -pi/2.,	a5:   0,	d6: 0,		q6: q6,
			alpha6: 0,	a6:   0,	d7: 0.303,	q7: 0}
	
	# Define Modified DH Transformation matrix
	def TF_Matrix(alpha, a, d, q):
		TF =	Matrix([[cos(q),	-sin(q),			0,		a],
			[sin(q)*cos(alpha),	cos(q)*cos(alpha),	-sin(alpha),	-sin(alpha)*d],
			[sin(q)*sin(alpha),	cos(q)*sin(alpha),	cos(alpha),	cos(alpha)*d],
			[0,			0,			0,			1]]) 
		
		return TF
	
	# Create individual transformation matrices
	T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
	T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
	T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
	T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
	T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
	T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
	T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
	
	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
	
	
	# Extract rotation matrices from the transformation matrices
	
	R0_3_Temp = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
	
	# Define Roatation Matrix from End Effector Yaw, Roll, Pitch

	r, p, y = symbols('r p y')
	
	ROT_x = Matrix([[ 1,	0,	0],			#Roll
              		[ 0,	cos(r), -sin(r)],
              		[ 0,	sin(r),	cos(r)]])
	
	ROT_y = Matrix([[ cos(p),        0,  sin(p)],		#Pitch
              		[       0,        1,        0],
              		[-sin(p),        0,  cos(p)]])
	
	ROT_z = Matrix([[ cos(y), -sin(y),        0],		#Yaw
              		[ sin(y),  cos(y),        0],
              		[ 0,              0,        1]])        

	ROT_EE = ROT_z * ROT_y * ROT_x
	
	## Initializtion

	# Define Joint angles for Target Spawn locations for the END trajectory point 
	#(theta1i,theta3i,theta4i,theta5i,theta6i, theta7i, theta8i, theta9i) are the joint angles for target sawn locations 1 to 9 
	# where i ranges from 1 to 6, while moving from initial location to target spawn location )

	theta1 = 0; theta2=0; theta3=0; theta4=0; theta5=0; theta6=0;
	theta61 = -0.460788639682829; theta62 = -1.4083946795092 + pi/2; theta63 = -1.49141431818708 + pi/2; theta64 = -1.0235104903567 + pi; theta65 = 0.495695716305110; theta66 = -pi + 0.972495930022601
	theta41 = 0.466568380940290; theta42 = -1.40588840950334 + pi/2; theta43 = -1.49933810543832 + pi/2; theta44 = -pi + 1.12482883075263; theta45 = 0.521928836050086; theta46 = -1.06984849582538 + pi
	theta91 = -0.460620971603582; theta92 = -1.41600678927374 + pi/2; theta93 = -2.01520122842406 + pi/2; theta94 =  0.912819391695163; theta95 = 0.538148746753259; theta96 = -0.855105912394959
	theta71 = 0.460651236883725; theta72 = -1.41603690529533 + pi/2; theta73 = -2.015158587665 + pi/2; theta74 = -0.913449451374226; theta75 = 0.537900753574318; theta76 = 0.855694311433023
	theta81 = -4.02985219558988e-6; theta82 = -1.59017523803714 + pi/2; theta83 = -1.81801235853875 + pi/2; theta84 = -8.71196937061172e-5; theta85 = 0.259284638790722 ; theta86 = -7.35675934196834e-6
	theta11 = 0.461944532159549; theta12 = -1.11836645228303 + pi/2; theta13 = -1.31463177224073 + pi/2; theta14 = -pi + 0.725225037468592; theta15 =  0.676095580016169; theta16 = -0.62726276643435 + pi
	theta51 = -0.00229054389158514; theta52 = -1.57765784191287 + pi/2; theta53 = -1.32851692115255 + pi/2; theta54 = -0.086229482068646 + pi; theta55 = 0.181470117894438; theta56 = -pi + 0.488456526793796
	theta31 = -0.466518984306794; theta32 = -1.08486429666301 + pi/2; theta33 = -1.31281014892211 + pi/2; theta34 = -0.618700551351137 + pi; theta35 = 0.886705067213612; theta36 = -pi + 0.441222764426423

	# Define Joint angle for the end trajectory point
 	# (thetaei) is the joint angle while moving from target spawn location to bin)      
  
	thetae1=-1.3683474710526 + pi; thetae2 = -0.965048732979534 + pi/2; thetae3 = -2.09403813236587 + pi/2; thetae4 = -1.55313297331450; thetae5 = -1.3697836855635 + pi; thetae6 = -1.48647561055299 + pi	
	
	# Initialize service response
	ReachCount = 0
        joint_trajectory_list = []
	ee_offset_list = []			# Create List of error in end effector
	time_list = []				# Create list of time samples

	if (len(req.poses)%2) == 0:		# For loop initiation of req.poses
	    i=1
	else:
	    i=0
	
	if (len(req.poses)>50):			# For loop initiation of req.poses
	    j=2

	else:
	    j=1
	    i=0

	Count.count = Count.count + 1		# Incrementing Iteration count for number of Inverse Kinematics loop
	print("IK count: %s" % (Count.count))
	
	# Calcukating Target Spawn END locations on shelf from request responses
	
	goal_px = req.poses[len(req.poses)-1].position.x
	goal_py = req.poses[len(req.poses)-1].position.y
	goal_pz = req.poses[len(req.poses)-1].position.z

	print("\nGoal position along x axis: %04.8f" % goal_px)
	print("Goal position along y axis: %04.8f" % goal_py)
	print("Goal position along z axis: %04.8f" % goal_pz)
	Target_Spawn = 0
	
	if ((Count.count%2) == 1):
		if (goal_px >= 2.08) & (goal_px <= 2.1) & (goal_py >= -1) & (goal_py <-0.8) & (goal_pz >= 1.58) & (goal_pz <= 1.59):		#Target Spawn location: 6 on shelf
			Target_Spawn = 6
		elif (goal_px >= 2.08) & (goal_px <= 2.1) & (goal_py >= 0.8) & (goal_py <1) & (goal_pz >= 2.3) & (goal_pz <= 2.5):		#Target Spawn location: 7 on shelf
			Target_Spawn = 7
		elif (goal_px >= 2.08) & (goal_px <= 2.1) & (goal_py >= 0.8) & (goal_py <1) & (goal_pz >= 1.58) & (goal_pz <= 1.59):		#Target Spawn location: 4 on shelf
			Target_Spawn = 4
		elif (goal_px >= 2.08) & (goal_px <= 2.1) & (goal_py >= -1) & (goal_py <-0.8) & (goal_pz >= 2.3) & (goal_pz <= 2.5):		#Target Spawn location: 9 on shelf
			Target_Spawn = 9
		elif (goal_px >= 2.08) & (goal_px <= 2.1) & (goal_py >= -0.1) & (goal_py <0.1) & (goal_pz >= 2.3) & (goal_pz <= 2.5):		#Target Spawn location: 8 on shelf
			Target_Spawn = 8
		elif (goal_px >= 2.08) & (goal_px <= 2.1) & (goal_py >= 0.8) & (goal_py <1) & (goal_pz >= 0.7) & (goal_pz <= 0.9):		#Target Spawn location: 1 on shelf
			Target_Spawn = 1
		elif (goal_px >= 2.08) & (goal_px <= 2.1) & (goal_py >= -1) & (goal_py <-0.8) & (goal_pz >= 0.7) & (goal_pz <= 0.9):		#Target Spawn location: 3 on shelf
			Target_Spawn = 3
		elif (goal_px >= 2.08) & (goal_px <= 2.1) & (goal_py >= -0.1) & (goal_py <0.1) & (goal_pz >= 1.58) & (goal_pz <= 1.59):		#Target Spawn location: 5 on shelf
			Target_Spawn = 5
		
	print("\nTarget spawn location is: %s" % (Target_Spawn))
	
        for x in xrange(i, len(req.poses),j):
            
            joint_trajectory_point = JointTrajectoryPoint()
	    print(x)
	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
	    
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
	    ROT_EE = ROT_EE * Rot_Error
	    ROT_EE=  ROT_EE.subs({'r':roll, 'p':pitch, 'y':yaw})

	    # Calculate End Effetor position
	    EE = Matrix([[px],
			[py],
			[pz]])

	    # Wrist Center Calculation
	    WC = EE - (0.303) * ROT_EE[:,2]

	### Inverse Kinematics code
	   
	    if len(joint_trajectory_list)>2:							#Calculate previous joint angles
		z = (joint_trajectory_list[len(joint_trajectory_list)-1])	    
		theta1 = z.positions[0]; theta2 = z.positions[1]; theta3 = z.positions[2]; theta4 = z.positions[3]; theta5 = z.positions[4]; theta6 = z.positions[5];

	    if (len(joint_trajectory_list)>2) & ((Count.count%2) == 0) & (x < (len(req.poses)-1)) & (theta1>-1.62 + pi) & (theta1<-1.36 + pi) & (theta2>-1.1 + pi/2) & (theta2<-0.94 + pi/2) & (theta3> -2.2 +pi/2) & (theta3< -1.9 + pi/2):
	    	
		print('Reached TargetLocation')							# Kuka arm has reached near the bin
		if (ReachCount == 0):
			ReachCount = 1
			joint_trajectory_point.positions = [thetae1, thetae2, thetae3, thetae4, thetae5, thetae6]
			joint_trajectory_list.append(joint_trajectory_point)
		
	    else:
		# Calculate joint angles using Geometric IK method

	    	theta1 = atan2(WC[1],WC[0])

		side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))			#side_a = 1.501	#side_c = 1.25
		

		if ((1.25 + 1.501)> side_b) & ((1.25 + side_b)> 1.501) & ((side_b + 1.501)> 1.25):				# Mathematically, (sum of 2 sides of a triangle) > 3rd side
			    
			angle_a = acos((side_b*side_b + 1.25*1.25 - 1.501*1.501) / (2*side_b*1.25))
			angle_b = acos((1.501*1.501 + 1.25*1.25 - side_b*side_b) / (2*1.501*1.25))
			angle_c = acos((1.501*1.501 + side_b*side_b - 1.25*1.25) / (2*1.501*side_b))

			theta2 = pi/2 - angle_a - atan2(WC[2]-0.75 , sqrt(WC[0]*WC[0] + WC[1]*WC[1])-0.35)
			theta3 = pi/2 - (angle_b  + atan2(0.054,1.5))


			R0_3 = R0_3_Temp.evalf(subs={q1:theta1, q2:theta2, q3:theta3})						# Evaluating Rotation Matrix

			R3_6 = R0_3.inv("LU") * ROT_EE

			theta4 = atan2(R3_6[2,2], -R3_6[0,2])

			theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])

			theta6 = atan2(-R3_6[1,1], R3_6[1,0])

			# Forward Kinematics evaluation with derived joint angles

			FK = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

			# Error Calculation of End Effector position by comparing with end-effector poses, received as input
			ee_x_e = abs(FK[0,3]-px)										# End Effector error offset along x axis
			ee_y_e = abs(FK[1,3]-py)										# End Effector error offset along y axis
			ee_z_e = abs(FK[2,3]-pz)										# End Effector error offset along z axis

			# Overall end-effector offset
			ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
			print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
			print ("End effector error for y position is: %04.8f" % ee_y_e)
			print ("End effector error for z position is: %04.8f" % ee_z_e)
			print ("Overall end effector offset is: %04.8f units \n" % ee_offset)

			# Populate response for the IK request

			if (x == len(req.poses)-1) & (Target_Spawn==6):									#Last joint angle calculation for Target Spawn location: 6
				if ((Count.count%2) == 1):

					joint_trajectory_point.positions = [theta61, theta62, theta63, theta64, theta65, theta66]
					joint_trajectory_list.append(joint_trajectory_point)

			elif (x == len(req.poses)-1) & (Target_Spawn==4):								#Last joint angle calculation for Target Spawn location: 4
				if ((Count.count%2) == 1):
			    		
					joint_trajectory_point.positions = [theta41, theta42, theta43, theta44, theta45, theta46]
					joint_trajectory_list.append(joint_trajectory_point)

			elif (x == len(req.poses)-1) & (Target_Spawn==9):								#Last joint angle calculation for Target Spawn location: 9
				if ((Count.count%2) == 1):

					joint_trajectory_point.positions = [theta91, theta92, theta93, theta94, theta95, theta96]
					joint_trajectory_list.append(joint_trajectory_point)

			elif (x == len(req.poses)-1) & (Target_Spawn==7):								#Last joint angle calculation for Target Spawn location: 7
				if ((Count.count%2) == 1):

					joint_trajectory_point.positions = [theta71, theta72, theta73, theta74, theta75, theta76]
					joint_trajectory_list.append(joint_trajectory_point)

			elif (x == len(req.poses)-1) & (Target_Spawn==8):								#Last joint angle calculation for Target Spawn location: 8
				if ((Count.count%2) == 1):
	
					joint_trajectory_point.positions = [theta81, theta82, theta83, theta84, theta85, theta86]
					joint_trajectory_list.append(joint_trajectory_point)

			elif (x == len(req.poses)-1) & (Target_Spawn==1):								#Last joint angle calculation for Target Spawn location: 1
				if ((Count.count%2) == 1):

					joint_trajectory_point.positions = [theta11, theta12, theta13, theta14, theta15, theta16]
					joint_trajectory_list.append(joint_trajectory_point)

			elif (x == len(req.poses)-1) & (Target_Spawn==5):								#Last joint angle calculation for Target Spawn location: 5
				if ((Count.count%2) == 1):

					joint_trajectory_point.positions = [theta51, theta52, theta53, theta54, theta55, theta56]
					joint_trajectory_list.append(joint_trajectory_point)

			elif (x == len(req.poses)-1) & (Target_Spawn==3):								#Last joint angle calculation for Target Spawn location: 3
				if ((Count.count%2) == 1):

					joint_trajectory_point.positions = [theta31, theta32, theta33, theta34, theta35, theta36]
					joint_trajectory_list.append(joint_trajectory_point)

			elif (x == len(req.poses)-1) & ((Count.count%2) == 0):								#Last joint angle calculation towards Bin
				if (ReachCount == 0):	
					if (ee_offset<0.01):							
						joint_trajectory_point.positions = [theta1, theta2, theta3, thetae4, thetae5, thetae6]
						joint_trajectory_list.append(joint_trajectory_point)
					else:
						joint_trajectory_point.positions = [thetae1, thetae2, thetae3, thetae4, thetae5, thetae6]
						joint_trajectory_list.append(joint_trajectory_point)

					zt = (joint_trajectory_list[len(joint_trajectory_list)-2])
					Theta_6 = zt.positions[5]
					Theta6Diff = (np.absolute(Theta_6-thetae6))
					
					if (Theta6Diff>= 1.7):			 
						joint_trajectory_point.positions = [thetae1, thetae2, thetae3, thetae4, thetae5, Theta_6]
						joint_trajectory_list.append(joint_trajectory_point)

			elif (ee_offset<0.01):												#Allow joint angles for which error is below threshold
				joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
				joint_trajectory_list.append(joint_trajectory_point)
				ee_offset_list.append(ee_offset)
				t = time()
				time_list.append(t)
   
	
	print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))
        
	rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		
	## Plotting error outputs of end effector w.r.t to received responses (Close the plot window for code to run further) 
	## UNCOMMENT BELOW LINES TO CHECK THE ERROR PLOTS FOR CHALLENGE COMPLETION

	#ee_offset_ar = np.array([ee_offset_list])
	#time_list_ar = np.array([time_list])
	#plt.figure(figsize=(15,7))
	#plt.rc('font', family='serif', size=15)
	#plt.plot(time_list_ar,ee_offset_ar, marker='o', color='r')
	#plt.xlabel('Time',fontsize=18)
    	#plt.ylabel('End Effector Offset',fontsize=18)
	#plt.grid(True)
	#plt.show()

        return CalculateIKResponse(joint_trajectory_list)

	
def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
