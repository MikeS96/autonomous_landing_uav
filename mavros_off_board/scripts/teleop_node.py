#!/usr/bin/env python3

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import TwistStamped # Use TwistStamped

from mavros_msgs.srv import SetMode  # Change fly mode of the vehicle
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import time

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e           u    i    o
   a    s    d           j    k    l
   z    x    c           m    ,    .

   Traslation         Rotation (Up-dowm)

r/v : increase/decrease max speeds by 10%
t/b : increase/decrease only linear speed by 10%
y/n : increase/decrease only angular speed by 10%

1 Change mode
2 Arm
3 Takeoff
4 Disarm
5 Land

CTRL-C to quit
"""

moveBindings = {

		# Traslations bindings
		'w':(-1,0,0,0,0,0),  # X, Y, Z, Roll, Pitch, Yaw,
		'q':(-1,-1,0,0,0,0),  
		'e':(-1,1,0,0,0,0),
		's':(0,0,0,0,0,0),
		'a':(0,-1,0,0,0,0),
		'd':(0,1,0,0,0,0),
		'x':(1,0,0,0,0,0),
		'z':(1,-1,0,0,0,0),
		'c':(1,1,0,0,0,0),

		# Altittude and rotation bindings
		'i':(0,0,1,0,0,0),
		',':(0,0,-1,0,0,0),
		'u':(0,0,1,0,0,1),
		'o':(0,0,1,0,0,-1),
		'k':(0,0,0,0,0,0),
		'j':(0,0,0,0,0,1),
		'l':(0,0,0,0,0,-1),
		'm':(0,0,-1,0,0,-1),
		'.':(0,0,-1,0,0,1),
	       }

speedBindings={
		'r':(1.1,1.1), # Linear, angular speed
		'v':(.9,.9),
		't':(1.1,1),
		'b':(.9,1),
		'y':(1,1.1),
		'n':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)

	# Publish in the setpont_velocity/cmd_vel This topic only control speeds.
	pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 1)
	rospy.init_node('teleop_node')  # Init node

	# Arming Service, True for Arming and False for disarm
	arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

	#T akeoff Service
	takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

	# Landing service
	landing_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

	# Changing mode service
	change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

	# Set linear and angular speed
	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	# Parameters initialization
	x = 0
	y = 0
	z = 0
	roll = 0 
	pitch = 0
	yaw = 0
	status = 0

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				roll = moveBindings[key][3]
				pitch = moveBindings[key][4]
				yaw = moveBindings[key][5]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15


			elif key == '2': # Key for arming
				rospy.wait_for_service('/mavros/cmd/arming')
				response = arming_cl(value = True)
				rospy.loginfo(response)

			elif key == '4': # Key for disarming
				rospy.wait_for_service('/mavros/cmd/arming')
				response = arming_cl(value = False)
				rospy.loginfo(response)

			elif key == '3': # Key for takingoff, with a altitude of 5 (Modify it)
				rospy.wait_for_service('/mavros/cmd/takeoff')
				response = takeoff_cl(altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0)
				rospy.loginfo(response)

			elif key == '5': # Key for Landing
				rospy.wait_for_service('/mavros/cmd/land')
				response = landing_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
				rospy.loginfo(response)

			elif key == '1': # Key for changing mode, change it based on your own needs.
				rospy.wait_for_service('/mavros/set_mode')
				response = change_mode(custom_mode="OFFBOARD")
				rospy.loginfo(response)
				
			else:
				x = 0
				y = 0
				z = 0
				roll = 0  
				pitch = 0
				yaw = 0
				if (key == '\x03'):
					break

			twist = TwistStamped() # Changed the constructor Twist for TwistStamped
			twist.header.frame_id = "1" 
			twist.header.stamp = rospy.Time.now()
			twist.twist.linear.x = x*speed
			twist.twist.linear.y = y*speed
			twist.twist.linear.z = z*speed #Change the object and method to use in linear and angular speed
			twist.twist.angular.x = roll*turn
			twist.twist.angular.y = pitch*turn
			twist.twist.angular.z = yaw*turn #Add roll, pitch, yaw varaibles to be published
			pub.publish(twist)

	except Exception as e:
		print(e)

	finally:
		twist = TwistStamped() # Changed the constructor Twist for TwistStamped
		twist.twist.linear.x = 0
		twist.twist.linear.y = 0
		twist.twist.linear.z = 0
		twist.twist.angular.x = 0
		twist.twist.angular.y = 0
		twist.twist.angular.z = 0
		pub.publish(twist)
		
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

