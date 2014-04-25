#!/usr/bin/env python

import rospy, tf, math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid

# Add additional imports for each of the message types used






#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
	#driveStraight(0.3, 0.6)
	#rotate(-pi/2)
	driveArc(0.5, 0.5, -2*pi)
	#rotate(4*pi)
	#driveStraight(0.3, 0.42)






#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):

	#0.035 for wheel
	#0.35 for base
	wheel = 0.035
	base = 0.35

	u = (u1 + u2) / 2
	w = (u1 - u2) / base

	t = rospy.get_time()
	twist = Twist()

	while ((rospy.get_time() - t) < time):
		twist = Twist()
		twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
		pub.publish(twist)

	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	pub.publish(twist)



#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):

	while (flag != 1):
		wait = 0

	xPosInit = pos.x
	yPosInit = pos.y
	xPosCurr = xPosInit
	yPosCurr = yPosInit

	twist = Twist()

	while (math.sqrt((xPosCurr - xPosInit) ** 2 + (yPosCurr - yPosInit) ** 2) < distance):

		twist = Twist()
		twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

		xPosCurr = pos.x
		yPosCurr = pos.y

	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	pub.publish(twist)	



	
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):

	global pi
	global orient
	current = 0
	last_yaw = 0
	yaw = 0

	while (flag != 1):
		wait = 0

	twist = Twist()

	euler = euler_from_quaternion(orient)
	last_yaw = euler[2]

	while (current <= 0 and current > angle):

		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -0.4
		pub.publish(twist)

		euler = euler_from_quaternion(orient)
		yaw = euler[2]


		if (yaw > 0 and last_yaw < 0):
			yaw = - (2 * pi + yaw)

		current -= math.sqrt((last_yaw - yaw)**2)

		yaw = euler[2]
		last_yaw = yaw

		print euler

	while (current >= 0 and current < angle):

		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.4
		pub.publish(twist)

		euler = euler_from_quaternion(orient)
		yaw = euler[2]

		if (yaw < 0 and last_yaw > 0):
			yaw = 2 * pi + yaw

		current += math.sqrt((last_yaw - yaw)**2)

		yaw = euler[2]
		last_yaw = yaw

		print euler

	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	pub.publish(twist)	





#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
	global pi
	global orient
	current = 0
	last_yaw = 0
	yaw = 0
	angularV = speed / radius

	while (flag != 1):
		wait = 0

	twist = Twist()

	euler = euler_from_quaternion(orient)
	last_yaw = euler[2]

	while (current <= 0 and current > angle):

		twist = Twist()
		twist.linear.x = -speed; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -angularV
		pub.publish(twist)

		euler = euler_from_quaternion(orient)
		yaw = euler[2]


		if (yaw > 0 and last_yaw < 0):
			yaw = - (2 * pi + yaw)

		current -= math.sqrt((last_yaw - yaw)**2)

		yaw = euler[2]
		last_yaw = yaw

		print euler

	while (current >= 0 and current < angle):

		twist = Twist()
		twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angularV
		pub.publish(twist)

		euler = euler_from_quaternion(orient)
		yaw = euler[2]

		if (yaw < 0 and last_yaw > 0):
			yaw = 2 * pi + yaw

		current += math.sqrt((last_yaw - yaw)**2)

		yaw = euler[2]
		last_yaw = yaw

		print euler

	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	pub.publish(twist)





#Odometry Callback function.
def read_odometry(msg):

	global flag
	global pos
	global orient
	global pi
	pi = 3.14159
	flag = 1
	pos = msg.pose.pose.position
	orient = [msg.pose.pose.orientation.x,
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w]
	#print pos
	#print orient

def read_grid(msg):
	global header
	global occupancyValue

	flag = 1
	occupancyValue = msg.data


#Bumper Event Callback function
def readBumper(msg):
	if (msg.state == 1):
		executeTrajectory()
		# What should happen when the bumper is pressed?



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
	pass # Delete this 'pass' once implemented









# This is the program's main function
if __name__ == '__main__':
	# Change this node name to include your username
	rospy.init_node('yzhou_lab3')


	# These are global variables. Write "global <variable_name>" in any other function
	#  to gain access to these global variables
	
	global pub
	global pose
	global odom_tf
	global odom_list
	global flag
	global pos
	global orient
	global pi

	flag = 0
	pi = 3.1415926
	
	# Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist) # Publisher for commanding robot motion
	sub = rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages
	occupationGrid = rospy.Subscriber('map', OccupancyGrid, read_grid, queue_size=1)

	#bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

	# Use this object to get the robot's Odometry 
	odom_list = tf.TransformListener()
	
	# Use this command to make the program wait for some seconds
	rospy.sleep(rospy.Duration(1, 0))



	print "Starting Lab 2"

	# Make the robot do stuff...
	#spinWheels(5, -5, 20)
	#driveStraight(20,4)
	#rotate(3*pi)
	#driveArc(0.5, 0.2, -pi)
	#executeTrajectory()
	#rotate(3*pi/4)

	#rospy.spin()
	print occupancyValue

	print "Lab 2 complete!"

