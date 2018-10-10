#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from yaml import dump
from os.path import join
from moveit_commander import MoveGroupCommander
import rospkg
rospack = rospkg.RosPack()

try:
	from yaml import CDumper as Dumper
except ImportError:
	from yaml import Dumper

class BlockTranslator:
	#J_Y_MAX = 0.34
	#J_Y_MIN = -0.17
	#J_X_MAX = 1.00
	#J_X_MIN = 0.44
	J_X_MIN = None
	J_Y_DIST = 0.37
	J_X_DIST = 0.770 #0.56

	#tui space measured from table:
	# topLeft = 0.95,0.106
	# bottomLeft = 0.95, 0.88
	# bottomRight = 0.06, 0.8
	# topRight = 0.09, 0.17
	T_Y_MAX = 0.8 #0.96
	T_Y_MIN = 0.17 #0.33
	T_X_MAX = 0.95 #0.77
	T_X_MIN = 0.09 #0.14

	T_Y_DIST = T_Y_MAX-T_Y_MIN
	T_X_DIST = T_X_MAX-T_X_MIN

	last_blocks = []

	def __init__(self):
		rospy.init_node("block_translator",anonymous=True)
		self.republisher = rospy.Publisher("jaco_blocks", String, queue_size=10)
		rospy.Subscriber("/blocks", String, self.republish)
		self.calibrate()
		rospy.spin()
	def calibrate(self,force=False):
		'''
		If passing j(x,y), assume that T(x,y) from /blocks corresponds and no other blocks on table
		'''
		if(rospy.has_param("J_X_MIN") and rospy.has_param("J_Y_MIN") and not force):
			rospy.loginfo("Already calibrated and force is not set. Not calibrating.")
			print("Table is already calibrated")
			self.J_X_MIN = rospy.get_param("J_X_MIN")
			self.J_X_MAX = self.J_X_MIN+self.J_X_DIST
			self.J_Y_MIN = rospy.get_param("J_Y_MIN")
			self.J_Y_MAX = self.J_Y_MIN+self.J_Y_DIST
			return

		arm = MoveGroupCommander("arm")
		arm.set_named_target("Home")
		arm.go()
		arm.set_num_planning_attempts(10)
		gripper = MoveGroupCommander("gripper")
		gripper.set_named_target("Open")
		gripper.go()
		rospy.sleep(5)
		gripper.set_named_target("Close")
		gripper.go()
		rospy.sleep(2)
		curpos = arm.get_current_pose()
		curpos.pose.position.x = 0.43
		curpos.pose.position.y = -0.45
		curpos.pose.position.z = 0.1
		curpos.pose.orientation.x = 0
		curpos.pose.orientation.y = 1
		curpos.pose.orientation.z = 0
		curpos.pose.orientation.w = 0
		
		arm.set_pose_target(curpos)
		arm.go()
		arm.go()
		curpos.pose.position.z = 0.02
		arm.set_pose_target(curpos)
		arm.go()
		gripper.set_named_target("Open")
		gripper.go()
		rospy.sleep(15)
		
		newpos = arm.get_current_pose()
		jx = newpos.pose.position.x
		jy = newpos.pose.position.y
		if len(self.last_blocks) < 1:
			print("no blocks. returning")
			rospy.loginfo("No Block for reference")
			return
		print("jx:",jx," jy:",jy," tx:",self.last_blocks[0]["x"]," ty:",self.last_blocks[0]["y"])
		if(len(self.last_blocks)>1):
			rospy.loginfo("Table must be empty to calibrate. Not calibrating.")
			print("Table is not empty")
			return
		t_x = self.last_blocks[0]["x"]
		t_y = self.last_blocks[0]["y"]
		print("WE MADE IT THUS FAR",t_x,t_y)
		x_prop,y_prop = self.scaleT2J(t_x,t_y)
		self.J_X_MIN = jx - x_prop*self.J_X_DIST
		self.J_X_MAX = self.J_X_MIN+self.J_X_DIST
		self.J_Y_MIN = jy - y_prop*self.J_Y_DIST
		self.J_Y_MAX = self.J_Y_MIN+self.J_Y_DIST
		rospy.set_param("J_X_MIN",self.J_X_MIN)
		rospy.set_param("J_Y_MIN",self.J_Y_MIN)		
		path = join(rospack.get_path('daarm'),'config','calibration_params.yaml')
		print("PATH:",self.J_X_MIN,self.J_Y_MIN)		
		with open(join(rospack.get_path('daarm'),'config','calibration_params.yaml'), 'w+') as dumpfile:
			dump({'J_X_MIN':self.J_X_MIN,'J_Y_MIN':self.J_Y_MIN},dumpfile, Dumper=Dumper)
			#dumpfile.write("THis is sparta!")
		newpos.pose.position.z = 0.3
		arm.set_pose_target(newpos)
		arm.go()
		arm.set_named_target("Home")
		arm.go()
		return

	def republish(self,message):
		blocks = eval(message.data)
		#print(blocks)
		self.last_blocks = blocks
		if self.J_X_MIN is None:
			return
		jaco_blocks = [self.translate(b) for b in blocks]
		msg = String()
		msg.data=str(jaco_blocks)
		self.republisher.publish(msg)
	
	def scaleT2J(self,x,y):
		return ((self.T_X_MAX-x)/self.T_X_DIST, (self.T_Y_MAX-y)/self.T_Y_DIST)


	def translate(self,block):
		#block = eval(block_)
		#x_prop = (self.T_X_MAX-block['x'])/self.T_X_DIST #the proportional distance from the left
		#y_prop = (self.T_Y_MAX-block['y'])/self.T_Y_DIST #the proportional distance from the bottom
		x_prop,y_prop = self.scaleT2J(block['x'],block['y'])
		jaco_x = x_prop*self.J_X_DIST + self.J_X_MIN
		jaco_y = y_prop*self.J_Y_DIST + self.J_Y_MIN
		return {"x":jaco_x, "y":jaco_y, "id":block['id']}

	

if __name__ == '__main__':
	try:
		bt = BlockTranslator()
	except rospy.ROSInterruptException:
		pass
