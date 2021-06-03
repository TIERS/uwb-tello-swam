#!/usr/bin/env python2

import time

from numpy.core.records import array
import rospy

import time
from threading import Thread

import queue

from swarm_controller import *

def main():

	rospy.init_node('tello_swarm_test_node', anonymous=True)

	num_of_drones = 4
	
	# initial_positions = [
	# 	[0,0],
	# 	[1,0]
	# ]

	# objective_positions = [
	# 	[4,4,2],
	# 	[3,4,2],
	# 	[5.5,4,2]
	# ]

	# objective_positions = [
	# 	[4.0,	4.0,	2],
	# 	[4.8,	4.0,	2],
	# 	[5.6,	4.0, 	2],
	# 	[4.8,	3.0,	2],
	# 	[4.9,	2.2, 	2],
	# 	[4.9, 	1.6, 	2]
	# ]

	# uwb_tags = [
	# 	"2D61",
	# 	"2B13",
	# 	"0CC8",
	# 	"2891",
	# 	"0D3B",
	# 	"28C0",
	# 	"2B26",
	# 	"8A09"
	# ]

	objective_positions = [
		[0,	0,	1],
		 [0.5, 0.0, 1.5],
		 [0, 1, 1],
		 [0, 3, 1]
	]

	objective_paths = [
		[[0,	0,	1], [1, 0, 1]],
		[[0.5, 0.0, 1.5], [1.5, 0.0, 1.5]],
		[[0, 1, 1], [1, 1, 1]],
		[[0, 3, 1], [1, 3, 1]]
	]
	
	import numpy as np
	
	uwb_tags = [
		"tello0",
		"tello1",
		"tello2",
		"tello3"
	]


	swarm = UWBTelloSwarm(num_of_drones, objective_positions, uwb_tags, objective_paths)

	# design a trajectory
	pos_traj = []
	ini_obj_position= swarm.get_agent_pos(0).copy()
	cur_obj_position = ini_obj_position.copy()
	# print(cur_obj_position)

	for i in range(0, 333):	
		cur_obj_position[0] += (0.3 * 0.01)
		# print(cur_obj_position)
		pos_traj.append(cur_obj_position.copy())

	# swarm.sequential_takeoff(delay=0.42)
	# swarm.parallel_takeoff()
	swarm.agent_takeoff(0)
	# swarm.parallel_takeoff_forward(delay=0.42)

	#swarm.parallel_takeoff_positioning()
	time.sleep(2)
	#swarm.parallel_takeoff()
	# set the trajectory
	swarm.set_trajectory(0, pos_traj)
	swarm.run() 
	#swarm.parallel_land()
	rospy.loginfo("Landing swarm.")


if __name__ == '__main__':
	main()
