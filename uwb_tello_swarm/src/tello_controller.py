#!/usr/bin/env python2

import sys
import time
import rospy
import numpy as np

from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Empty

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from tello_driver.msg import TelloStatus

MAX_POS_ERROR = 0.1
BASE_SPEED = 0.5


class TelloController():
	'''
		Single Tello EDU Controller

		Contains utility functions for
		  - Takeoff
		  - Land
		  - Navigate to position

		For navigation:
		  - Uses onboard odometry if no external positioning is given.
		  - Relies on pose published to "pos_topic" otherwise.
		    (currently only for position)
	'''

	def __init__(self, i, ros_ns="tello", pos_topic=None, takeoff_height=1.2, external_z=True) :
		'''
			Controller to be used by TelloSwarm
			(can also be used standalone)

			Params:
			  - takeoff_height: default takeoff
			  - external_z: use external pos for height (if true) or odom
		'''

		self.id = i
		self.ns = ros_ns

		self.x0 = None
		self.y0 = None
		self.z0 = None

		self.x = None
		self.y = None
		self.z = None

		self.pos = np.array([None, None, None])
		self.obj = np.array([None, None, takeoff_height])

		self.external_z = external_z

		self.running = False
		self.battery = -1

		self.land_pub = rospy.Publisher('/{}/land'.format(self.ns), Empty, queue_size=10)
		self.takeoff_pub = rospy.Publisher('/{}/takeoff'.format(self.ns), Empty, queue_size=10)
		self.emergency_pub = rospy.Publisher('/{}/emergency'.format(self.ns), Empty, queue_size=10)

		self.vel_pub = rospy.Publisher('/{}/cmd_vel'.format(self.ns), Twist, queue_size=10)
		self.odom_sub = rospy.Subscriber('/{}/odom'.format(self.ns), Odometry, self.cb_odom)
		self.tello_status = rospy.Subscriber('/{}/status'.format(self.ns), TelloStatus, self.cb_status)

		if pos_topic :
			print("Subscribing to {}".format(pos_topic))
			self.pos_topic = pos_topic
			self.position_sub = rospy.Subscriber(self.pos_topic, Pose, self.cb_positioning)
			sys.stdout.write('Waiting for external position...')
			sys.stdout.flush()

		time.sleep(3)		
		i = 0
		rate = rospy.Rate(5)
		while i < 42 :
			sys.stdout.write('.')
			sys.stdout.flush()
			if self.x is not None and self.y is not None and self.z is not None :
				sys.stdout.write('... position locked!\n\n')
				sys.stdout.flush()
				rospy.loginfo('Tello controller for drone {} in network namespace {} is ready.'.format(self.id, self.ns))
				self.x0 = self.x
				self.y0 = self.y
				self.z0 = self.z
				self.obj_x = self.x
				self.obj_y = self.y
				break
			else :	
				rate.sleep()
				i += 1
		 
		if i == 42 :
			sys.stdout.write('\n\n')
			sys.stdout.flush()
			rospy.loginfo('Tello controller for drone {} timed out.'.format(self.id))


	def cb_odom(self, msg) :
		'''
			Tello odom subscriber.
			Only updates positions if there is no external ref.
		'''
		if not self.pos_topic :
			self.x = msg.pose.pose.position.x
			self.y = msg.pose.pose.position.y
			self.z = msg.pose.pose.position.z

		if not self.external_z :
			self.z = msg.pose.pose.position.z
			self.set_speed()
	
	def cb_status(self, msg) :
		'''

		'''
		self.battery = msg.battery_percentage
	
	def cb_positioning(self, msg) :
		'''
			External positioning subscriber
		'''
		self.x = msg.position.x
		self.y = msg.position.y
		if self.external_z :
			self.z = msg.position.z
		
		self.pos = np.array([self.x, self.y, self.z])

		if self.running and self.obj.all() and self.pos.all() :
			self.set_speed()

	def set_objective(self, pos) :
		'''
			Sets objective position
			(TODO: add orientation)
		'''
		assert len(pos) == 3
		self.obj = np.array(pos)
		# self.obj_x = x#-self.x0
		# self.obj_y = y#-self.y0
		# self.obj_z = z

	def set_speed(self) :
		'''
			Calculates speed to current objective position.

			Currently called from the subscriber of odom or position.
		'''

		# Message template
		twist = Twist()

		#
		#	VERY simple approach for proof of concept
		#
		#		- First move to appropriate altitude
		#		- Then move in XY plane
		#

		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0

		diff = self.obj - self.pos
		[twist.linear.x, twist.linear.y, twist.linear.z] = BASE_SPEED * diff / np.linalg.norm(diff)
		
		# if self.x < self.obj_x - MAX_POS_ERROR :
		# 	twist.linear.x = BASE_SPEED

		# elif self.x > self.obj_x + MAX_POS_ERROR :
		# 	twist.linear.x = -BASE_SPEED

		# if self.y < self.obj_y - MAX_POS_ERROR :
		# 	twist.linear.y = BASE_SPEED

		# elif self.y > self.obj_y + MAX_POS_ERROR :
		# 	twist.linear.y = -BASE_SPEED

		# No rotation	
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0

		# Publish
		# rospy.loginfo(str(twist))
		self.vel_pub.publish(twist)
		

	def takeoff(self) :
		'''
			Publishes a takeoff message
			five times in 0.1s intervals.
		'''
		rospy.loginfo("Tello{} attempting take off...".format(self.id))
		msg = Empty()
		for _ in range(2) :
			self.takeoff_pub.publish(msg)
			time.sleep(0.2)

	def land(self) :
		'''
			Publishes a land message
			five times in 0.1s intervals.

			After 3s turns off the motors 
			(just in case).
		'''
		msg = Empty()
		for _ in range(1) :
			self.land_pub.publish(msg)
			# time.sleep(0.1)
		# time.sleep(3)
		# self.emergency_pub.publish(msg)

if __name__ == '__main__':
	raise Exception("Do not run this directly!")
