#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

throttles={
		'w':(1.1,),
		's':(.9,),
		'g':(0.0,),
	      }

atti={
		'j':(1,0,0,0),
		'k':(0,1,0,0),
		'l':(0,0,1,0),
		'i':(0,0,0,1),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1
throttle = .5

def vels(throttle,turn):
	return "currently:\tthrottle %s\tturn %s " % (throttle,turn)

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_key', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	x = 0
	y = 0
	z = 0
	th = 0
	status = 0
	twist = Twist()
	try:
		# print msg
		# print vels(speed,turn)
		while(1):
			key = getKey()

			# if key in moveBindings.keys():
			# 	x = moveBindings[key][0]
			# 	y = moveBindings[key][1]
			# 	z = moveBindings[key][2]
			# 	th = moveBindings[key][3]
			# elif key in speedBindings.keys():
			# 	speed = speed * speedBindings[key][0]
			# 	turn = turn * speedBindings[key][1]

			# 	print vels(speed,turn)
			# 	if (status == 14):
			# 		print msg
			# 	status = (status + 1) % 15
			if key in throttles.keys():
				throttle = throttle * throttles[key][0]
				if (throttle<0.1):
					throttle = 0.5
			elif key in atti.keys():
				# print(1)
				twist.angular.x += atti[key][0]
				twist.angular.y += atti[key][1]
				twist.angular.z += atti[key][2]
				twist.linear.z += atti[key][3]
				pass

			print vels(throttle,speed)
			if (status == 14):
				print msg
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			# twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			# twist.linear.x = speed; twist.linear.y = turn; twist.linear.z = z*speed
			# twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			twist.linear.x = throttle
			pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0.5; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


