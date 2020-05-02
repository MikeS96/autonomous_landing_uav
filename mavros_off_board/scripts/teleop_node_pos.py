#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from mavros_msgs.msg import PositionTarget # Use PositionTarget
from mavros_msgs.srv import SetMode  
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import time

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
       w                    i    
   a   s    d           j   k   l
       x                    ,    

   Traslation         Rotation (Up-dowm)

r/v : increase/decrease max speeds by 10%
t/b : increase/decrease only linear speed by 10%
y/n : increase/decrease only angular speed by 10%
u/m : increase/decrease only altitude  by 10%

1 Change mode
2 Arm
3 Takeoff
4 Disarm
5 Land

CTRL-C to quit
"""

moveBindings = {

		# Traslations bindings
		'w':(1,0,1,0),  # Vx, Vy, Z and Yaw_rate
		'a':(0,-1,1,0),
		'd':(0,1,1,0),
		'x':(-1,0,1,0),



		'k':(0,0,1,0),
		'j':(0,0,1,1),
		'l':(0,0,1,-1),
	       }

speedBindings={
		'r':(1.1,1.1,1), # Reduce speed and turn
		'v':(.9,.9,1),
		't':(1.1,1,1), # Reduce only speed
		'b':(.9,1,1),
		'y':(1,1.1,1), # Reduce only turn
		'n':(1,.9,1),
		'i':(1,1,1.1), # Reduce only heigh
		',':(1,1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.35)
	if rlist: 
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	# Publish in the /mavros/setpoint_raw/local This topic controls position and speed
	pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size = 10)
	rospy.init_node('teleop_node_local')  # Init node

	#Arming Service, True for Arming and False for disarm
	arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

	#Takeoff Service
	takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

	#Landing service
	landing_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

	#Changing mode service
	change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

	# Set linear and angular speed
	speed = rospy.get_param("~speed", 1.5)
	turn = rospy.get_param("~turn", 1.0)
	# Parameters initialization
	alt = 2
	vx = 0
	vy = 0
	z = 0
	yaw_rate = 0
	status = 0
	mask = 1987
	last_z = 0 #Variable to save the last Z pos
	rate = rospy.Rate(10)

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				vx = moveBindings[key][0]
				vy = moveBindings[key][1]
				z = moveBindings[key][2]
				yaw_rate = moveBindings[key][3]
				last_z = z
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				alt = alt * speedBindings[key][2]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
					print(alt)
				status = (status + 1) % 15


			elif key == '2': # Key for arming
			       rospy.wait_for_service('/mavros/cmd/arming')
                               response = arming_cl(value = True)
			       rospy.loginfo(response)

    			elif key == '4': # Key for disarming
   			       rospy.wait_for_service('/mavros/cmd/arming')
                               response = arming_cl(value = False)
			       rospy.loginfo(response)

    			elif key == '3': # Key for takeoff, with a altitude of 3
   			       rospy.wait_for_service('/mavros/cmd/takeoff')
                               response = takeoff_cl(altitude=3, latitude=0, longitude=0, min_pitch=0, yaw=0)
			       rospy.loginfo(response)

			elif key == '5': # Key for Landing, with a altitude of 0 
   			       rospy.wait_for_service('/mavros/cmd/land')
                               response = landing_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
			       rospy.loginfo(response)

			elif key == '1': # Key for changing mode
   			       rospy.wait_for_service('/mavros/set_mode')
                               response = change_mode(custom_mode="OFFBOARD")
			       rospy.loginfo(response)
				
			else:
				vx = 0
				vy = 0
				z = last_z
				yaw_rate = 0
				if (key == '\x03'):
					break

			post = PositionTarget() # Use PositionTarget publisher
			post.header.frame_id = "home"
   		        post.header.stamp = rospy.Time.now()
    		        post.coordinate_frame = 8 # pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    			post.type_mask  = mask
			post.position.z = z*alt; # Altittude Position
			post.velocity.x = vy*speed; post.velocity.y = vx*speed; # X and Y velocity
			post.yaw_rate = yaw_rate*turn # Yaw rate
			pub.publish(post)
			rate.sleep()

	except Exception as e:
		print(e)

	finally:
		post = PositionTarget() # Use PositionTarget publisher
		post.header.frame_id = "home"
   		post.header.stamp = rospy.Time.now()
    		post.coordinate_frame = 8 # pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    		post.type_mask  = mask
		post.position.z = 1; # Altittude Position
		post.velocity.x = 0; post.velocity.y = 0; # X and Y velocity
		post.yaw_rate = 0 # Yaw rate
		pub.publish(post)
		rate.sleep()

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

