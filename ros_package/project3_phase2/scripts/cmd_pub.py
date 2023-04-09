#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import csv
import pandas as pd
import time

# Wheel radius and distance between robot wheels in meters
wheel_r = 0.033
wheel_base = 0.16

# Multiplication factors to obtain good results in open loop configuration
total_mul = 1
time_mul = 3 / total_mul
speed_mul = 0.45 * total_mul

# Function to read rpm values from csv file and publish one-by-one to the robot
def cmd_publisher():
	# Initialize the node
	rospy.init_node('turtlebot3_controller')
	rate = rospy.Rate(1/time_mul)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=20)		# Creating a publisher object that will publish msgs of type cmd_vel to the turtlebot3
	idx = 0
	time.sleep(2)  # Delay to let the launch file start everything

	# iterate through the csv file and update command at a constant rate
	try:
		# Read CSV file
		data = pd.read_csv("~/catkin_ws/src/project3_phase2/data/rpm.csv", delimiter=',', header=None)
		
		# Looping till ros core is interrupted or until the whole csv file has been traversed
		while not rospy.is_shutdown() and idx < len(data):
			# Scaling the rpms as per speed multiplier
			l_rpm = data.loc[[idx]][0]*speed_mul
			r_rpm = data.loc[[idx]][1]*speed_mul

			# Printing the current rpms for debugging purposes
			print(idx, l_rpm.values, r_rpm.values)

			# Calculating the bot linear and angular velocities
			lin_vel = (wheel_r*(l_rpm + r_rpm))/2.0 
			ang_vel = (r_rpm - l_rpm)*wheel_r/wheel_base

			# Updating the message
			robot_vel = Twist()
			robot_vel.linear.x = lin_vel
			robot_vel.linear.y = 0.0
			robot_vel.linear.z = 0.0
			robot_vel.angular.x = 0.0
			robot_vel.angular.y = 0.0
			robot_vel.angular.z = ang_vel
			pub.publish(robot_vel)
			rate.sleep()
			idx += 1

	except:
		print("Exception error")

	finally:
		robot_vel = Twist()
		robot_vel.linear.x = 0.0
		robot_vel.linear.y = 0.0
		robot_vel.linear.z = 0.0
		robot_vel.angular.x = 0.0
		robot_vel.angular.y = 0.0
		robot_vel.angular.z = 0.0
		pub.publish(robot_vel)



if __name__=="__main__":
	try:
		cmd_publisher()

	except rospy.ROSInterruptException():
		pass