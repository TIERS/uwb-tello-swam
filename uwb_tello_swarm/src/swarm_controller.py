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

from tello_controller import *

class UWBTelloSwarm():

	def __init__(self, num, objective_pos, uwb_tags) :
		'''
			Swarm controller
		'''

		assert num > 0
		assert num <= len(objective_pos)

		self.uwb_tags = uwb_tags

		self.drones = []
		for i in range(num) :
			self.drones.append(
				TelloController(i, 
					ros_ns = "tello{}".format(i), 
					pos_topic = "/dwm1001/tag/{}/position".format(self.uwb_tags[i]),
					takeoff_height=1.2, 
					external_z=True
				)
			)
			self.drones[i].set_objective(objective_pos[i])#[0], objective_pos[i][1], objective_pos[i][2])

		# Delay to
		time.sleep(2)
		rospy.loginfo("Swarm ready.")
	
	def parallel_takeoff(self) :
		'''
			Command all drones to take off simultanously
			(minimal delay if the number is very large)
		'''
		for drone in self.drones :
			drone.takeoff()

	def sequential_takeoff(self, delay=0.23) :
		'''
			Command drones to take off one by one
		'''
		rospy.loginfo("Taking off sequentially...")
		for drone in self.drones :
			drone.takeoff()
			time.sleep(delay)

	def parallel_land(self) :
		'''
			Command all drones to land simultanously
			(minimal delay if the number is very large)
		'''
		for drone in self.drones :
			drone.land()

	def sequential_land(self, delay=0.23) :
		'''
			Command drones to land one by one
		'''
		for drone in self.drones :
			drone.land()
			time.sleep(delay)

	def run(self) :
		'''
			Rospy spin while drones go to
			their objective positions
		'''
		rospy.loginfo("Ready to run!")
		for drone in self.drones :
			drone.running = True
		
		rate = rospy.Rate(5)
		while not rospy.is_shutdown() :
			batteries = ""
			for drone in self.drones :
				batteries += " -- " + str(drone.battery)
			sys.stdout.write('\r Swarm battery levels: {}'.format(batteries))
			rate.sleep()
		# rospy.spin()
		self.runnign = False

if __name__ == '__main__':
	raise Exception("Do not run this directly!")
