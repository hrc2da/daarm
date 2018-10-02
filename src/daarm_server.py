#!/usr/env/bin python

import roslib
roslib.load_manifest('daarm')
import rospy
import actionlib

from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String
from daarm.msg import BuildConfigAction


class DAArmServer:
	'''
	actionlib server that exposes a service to build a config
	goal: String bitstring
	result: String bitstring
	feedback: number of actions completed out of the total
	'''
	def __init__(self):
		self.server = actionlib.SimpleActionServer('build_config', BuildConfigAction, self.execut, False)
		self.server.start()

	def execute(self, goal):
		#translate the goal to a series of actions based on current config

		#for each action, plan and then execute

		#update the action's success

		#send completed message
		
		self.server.set_succeeded()


if __name__ == '__main__':
	rospy.init_node('daarm_server')
	server = DAArmServer()
	rospy.spin()


