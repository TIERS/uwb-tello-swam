#!/usr/bin/env python2

#from _typeshed import NoneType
from logging import RootLogger
import sys
import time
import rospy
import math
import numpy as np

from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Empty

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from tello_driver.msg import TelloStatus

from tello_controller import *


SWARM_SPEED_STOP_ERROR = 0.05


class UWBTelloSwarm():

    def __init__(self, num, objective_pos, uwb_tags, objective_paths = None):
        '''
                Swarm controller
        '''

        assert num > 0
        assert num <= len(objective_pos)

        self.uwb_tags = uwb_tags
        self.bearing31 = None
        self.bearing32 = None
        self.bearing41 = None
        self.bearing42 = None
        self.bearing43 = None

        self.drones = []
        for i in range(num):
            self.drones.append(
                TelloController(i,
                                ros_ns="tello{}".format(i),
                                pos_topic="/vrpn_client_node/{}/pose".format(
                                    self.uwb_tags[i]),
                                takeoff_height=1.2,
                                external_z=True
                                )
            )
            # [0], objective_pos[i][1], objective_pos[i][2])
            self.drones[i].set_objective(objective_pos[i])

        # Delay to
        time.sleep(2)
        rospy.loginfo("Swarm ready.")

    # Below is original
    # def parallel_takeoff(self) :
    # 	'''
    # 		Command all drones to take off simultanously
    # 		(minimal delay if the number is very large)
    # 	'''
    # 	for drone in self.drones :
    # 		drone.takeoff()

    # 	for drone in self.drones :
    # 		drone.positioning()

    def parallel_takeoff(self):
        '''
            take off the drones
        '''
        for drone in self.drones:
            drone.takeoff()
    

    def agent_takeoff(self, i):
        '''
            take off the drones
        '''
        self.drones[i].takeoff()
    
    def get_agent_pos(self, i):
        '''
        get the current position of agent i
        '''
        return self.drones[i].pos
 #   def firstleader_trajectorytracking(self):
  #      '''
  #          the first leader tracks a desired trajectory
 #       '''
  #      steps=100
   #     i=0
    #    while(i<100)
     #      parallel_takeoff_positioning(self)

    # def calculation_angle123(self):
    #     if np.linalg.norm(self.drones[0].pos-self.drones[1].pos)> 0.005:
    #         bearing21 = (self.drones[0].pos-self.drones[1].pos)/ \
    #             np.linalg.norm(self.drones[0].pos-self.drones[1].pos)
    #     else:
    #         bearing21 = [0, 0, 0]
       
    #     if np.linalg.norm(self.drones[2].pos-self.drones[1].pos)> 0.005:
    #         bearing23 = (self.drones[2].pos-self.drones[1].pos)/ \
    #             np.linalg.norm(self.drones[2].pos-self.drones[1].pos)
    #     else:
    #         bearing23 = [0, 0, 0] 
        
    #     #bearing21tr = bearing21.transpose()
    #     print(bearing23)
    #     print(bearing21)
    #     print(np.dot(bearing21,bearing23))
    #     return math.acos(np.dot(bearing21,bearing23))

    # def calculation_angle132(self):
    #     if np.linalg.norm(self.drones[0].pos-self.drones[2].pos) > 0.001:     
    #         bearing31 = (self.drones[0].pos-self.drones[2].pos) / \
    #             np.linalg.norm(self.drones[0].pos-self.drones[2].pos)
    #     else:
    #         bearing31 = [0, 0, 0] 
    #     if np.linalg.norm(self.drones[1].pos-self.drones[2].pos) > 0.001:
    #         bearing32 = (self.drones[1].pos-self.drones[2].pos) / \
    #             np.linalg.norm(self.drones[1].pos-self.drones[2].pos)    
    #     else:
    #         bearing32 = [0, 0, 0]
    #     self.bearing31 = bearing31
    #     self.bearing32 = bearing32
    #   #  bearing31tr = bearing31.transpose()
    #     print(bearing32)
    #     print(bearing31)
    #     print(np.dot(bearing31,bearing32))
    #     return math.acos(np.dot(bearing31,bearing32))
    
    def calculation_angle123(self):
        
        bearing21 = (self.drones[0].pos-self.drones[1].pos)/ \
            np.linalg.norm(self.drones[0].pos-self.drones[1].pos)
       
        bearing23 = (self.drones[2].pos-self.drones[1].pos)/ \
            np.linalg.norm(self.drones[2].pos-self.drones[1].pos)
        
        
        #bearing21tr = bearing21.transpose()
        #print(bearing23)
        #print(bearing21)
        #print(np.dot(bearing21,bearing23))
        return math.acos(np.dot(bearing21,bearing23))

    def calculation_angle132(self):     
        bearing31 = (self.drones[0].pos-self.drones[2].pos) / \
            np.linalg.norm(self.drones[0].pos-self.drones[2].pos)
        
        bearing32 = (self.drones[1].pos-self.drones[2].pos) / \
            np.linalg.norm(self.drones[1].pos-self.drones[2].pos)    
        
        self.bearing31 = bearing31
        self.bearing32 = bearing32
      #  bearing31tr = bearing31.transpose()
        #print(bearing32)
        #print(bearing31)
        #print(np.dot(bearing31,bearing32))
        return math.acos(np.dot(bearing31,bearing32))
    
    def calculation_angle142(self):
        
        bearing41 = (self.drones[0].pos-self.drones[3].pos)/ \
            np.linalg.norm(self.drones[0].pos-self.drones[3].pos)
       
        bearing42 = (self.drones[1].pos-self.drones[3].pos)/ \
            np.linalg.norm(self.drones[1].pos-self.drones[3].pos)
        
        self.bearing41 = bearing41
        self.bearing42 = bearing42
        return math.acos(np.dot(bearing41,bearing42))
    
    def calculation_angle421(self):
        
        bearing24 = (self.drones[3].pos-self.drones[1].pos)/ \
            np.linalg.norm(self.drones[3].pos-self.drones[1].pos)
       
        bearing21 = (self.drones[0].pos-self.drones[1].pos)/ \
            np.linalg.norm(self.drones[0].pos-self.drones[1].pos)
        
        #self.bearing24 = bearing24
        #self.bearing21 = bearing21
        return math.acos(np.dot(bearing24,bearing21))

    def calculation_angle423(self):
        
        bearing24 = (self.drones[3].pos-self.drones[1].pos)/ \
            np.linalg.norm(self.drones[3].pos-self.drones[1].pos)
       
        bearing23 = (self.drones[2].pos-self.drones[1].pos)/ \
            np.linalg.norm(self.drones[2].pos-self.drones[1].pos)
        
        # self.bearing41 = bearing41
        # self.bearing42 = bearing42
        return math.acos(np.dot(bearing24,bearing23))

    def swarmcontrol_drone2(self, i):
        delta = [0.7,	0.7,  -1]
        diff2 = self.drones[0].pos - \
            self.drones[1].pos - delta
        drone = self.drones[i]
        if drone.is_connection():
            drone.set_speed_step(diff2)
            

    def swarmcontrol_drone3(self, i):
        desired123 = math.pi/3
        desired132 = math.pi/3
        error123 = self.calculation_angle123()-desired123
        error132 = self.calculation_angle132()-desired132
        # if np.linalg.norm(self.drones[1].pos-self.drones[2].pos) > 0.005:
        #     bearing32 = self.drones[1].pos-self.drones[2].pos / \
        #         np.linalg.norm(self.drones[1].pos-self.drones[2].pos)    
        # else:
        #     bearing32 = 0 
        #bearing31 = self.drones[0].pos-self.drones[2].pos
        diff3 = error123*self.bearing31-error132*self.bearing32  
        drone = self.drones[i]
        if drone.is_connection():
            drone.set_speed_step(diff3)

    def swarmcontrol_drone4(self,i):
        desired142 = math.pi/3
        desired421 = math.pi/3
        desired423 = math.pi/3
        error142 = self.calculation_angle142()-desired142
        error421 = self.calculation_angle421()-desired421
        error423 = self.calculation_angle423()-desired423
        bearing43 = (self.drones[2].pos-self.drones[3].pos)/ \
            np.linalg.norm(self.drones[2].pos-self.drones[3].pos) #bearing43 not used in the calculations of the angles
        diff4 = -error142*self.bearing42+error421*self.bearing41+error423*bearing43
        drone = self.drones[i]
        if drone.is_connection():
            drone.set_speed_step(diff4)



    # ORIGINAL:
    # def parallel_takeoff_positioning(self):
    #     '''
    #         Command all drones to take off simultanously
    #         (minimal delay if the number is very large)
    #     '''

    #     for drone in self.drones:
    #         drone.takeoff()

    #     dist = np.ones(len(self.drones))

    #     while(max(dist) > SWARM_SPEED_STOP_ERROR):
    #         i = 0

    #         for drone in self.drones:
    #             diff = drone.obj[:2] - drone.pos[:2]
    #             dist_element = np.linalg.norm(diff)
    #             dist[i] = dist_element
    #             i += 1
    #             if drone.is_connection():
    #                 drone.set_speed_step(diff)

    def parallel_takeoff_positioning(self):
        '''
            Command all drones to take off simultanously
            (minimal delay if the number is very large)
        '''

        for drone in self.drones:
            drone.takeoff()

        dist = np.ones(len(self.drones))

        while(max(dist) > SWARM_SPEED_STOP_ERROR):
            i = 0

            for drone in self.drones:
                diff = drone.obj[:2] - drone.pos[:2]
                dist_element = np.linalg.norm(diff)
                dist[i] = dist_element
                i += 1
                if drone.is_connection():
                    drone.set_speed_step(diff)

    # original
    # def agent_positioning(self, i):
    #     '''
    #         Command the drone i to take off simultanously
    #     '''

    #     drone = self.drones[i]

    #     diff = drone.obj - drone.pos

    #     if drone.is_connection():
    #         drone.set_speed_step(diff)
    
    def agent_positioning(self, i):
        '''
            Command the drone i to take off simultanously
        '''

        drone = self.drones[i]

        diff = drone.obj - drone.pos

        if drone.is_connection():
            drone.set_speed_step(diff)

    def agent_path_following(self, i):
        drone = self.drones[i]
        diff = drone.obj - drone.pos


    def parallel_takeoff_forward(self, delay=0.23):
        '''
                Command all drones to take off simultanously
                (minimal delay if the number is very large)
        '''
        rospy.loginfo("Taking off sequentially...")
        cmd = [1, 0, 0]
        for drone in self.drones:
            drone.takeoff()
            time.sleep(delay)
            drone.forward(cmd)
            drone.land()

    def sequential_takeoff_positioning(self, delay=0.23):
        '''
                Command drones to take off one by one
        '''
        rospy.loginfo("Taking off sequentially...")
        for drone in self.drones:
            drone.takeoff()
            time.sleep(delay)

        for drone in self.drones:
            drone.positioning()
            time.sleep(delay)

    def parallel_land(self):
        '''
                Command all drones to land simultanously
                (minimal delay if the number is very large)
        '''
        for drone in self.drones:
            drone.land()

    def sequential_land(self, delay=0.23):
        '''
                Command drones to land one by one
        '''
        for drone in self.drones:
            drone.land()
            time.sleep(delay)

    # def run(self):
    #     '''
    #             Rospy spin while drones go to
    #             their objective positions
    #     '''
    #     rospy.loginfo("Ready to run!")
    #     for drone in self.drones:
    #         drone.running = True

    #     rate = rospy.Rate(5)
    #     while not rospy.is_shutdown():
    #         batteries = ""
    #         for drone in self.drones:
    #             batteries += " -- " + str(drone.battery)
    #         sys.stdout.write('\r Swarm battery levels: {}'.format(batteries))

    #         rate.sleep()
    #     # rospy.spin()
    #     self.running = False
    def set_trajectory(self, i, pos_traj):
                    # set the path for tello i
            self.drones[i].set_path(pos_traj)

    def run(self):
        '''
                Rospy spin while drones go to
                their objective positions
        '''
        rospy.loginfo("Ready to run!")
        for drone in self.drones:
            drone.running = True

        rate = rospy.Rate(100)

        index = 0

        num_point_traj = len(self.drones[0].obj_path)

        while not rospy.is_shutdown():
            if(self.drones[0].obj_path is not None):
                self.drones[0].set_objective(self.drones[0].obj_path[index])
                print("current objective: {}".format(self.drones[0].obj))
            self.agent_positioning(0)
            # self.agent_positioning(1)
         #   self.swarmcontrol_drone2(1)
         #   self.swarmcontrol_drone3(2)
         #   self.swarmcontrol_drone4(3)
            if(index < num_point_traj - 1):
                index += 1

            # batteries = ""
            # for drone in self.drones:
            #     batteries += " -- " + str(drone.battery)
            # sys.stdout.write('\r Swarm battery levels: {}'.format(batteries))

            rate.sleep()

        # rospy.spin()
        self.running = False


if __name__ == '__main__':
    raise Exception("Do not run this directly!")
