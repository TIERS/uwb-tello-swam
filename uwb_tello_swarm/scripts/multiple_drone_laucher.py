#! /usr/bin/python

import roslaunch
import rospy
import time

# <arg name="namespace"			default="tello" />
# <arg name="tello_ip" 			default="192.168.10.1" />
# <arg name="local_cmd_client_port"	default="8890" />
# <arg name="local_vid_server_port"	default="6038" />
# <arg name="tello_cmd_server_port"	default="8889" />

num_nodes = 3

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch_file = "tello_node_multi.launch"
launch_files = []
for num in range(num_nodes):
   cli_args = ['tello_driver', launch_file, 'namespace:=tello{}'.format(num), 
               'tello_ip:=192.168.1.{}'.format(150+num), 'local_cmd_client_port:={}'.format(8890+num),
               'local_vid_server_port:={}'.format(6038 + num), 'tello_cmd_server_port:={}'.format(8889)]
   roslaunch_args = cli_args[2:]
   roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
   print(cli_args, " ===>>>  ", num)
   
   launch_files=[(roslaunch_file, roslaunch_args)]

   parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

   parent.start()

rospy.spin()
