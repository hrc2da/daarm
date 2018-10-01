#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time

class ConfigToAction:

	BLOCKS = ["A","B","C","D","E","F","G","H","I","J","K","L"]

	ORBITS = [[-0.235,-0.14],[-0.3175,-0.235],[-0.4191,-0.3175],[-0.51435,-0.4191],[-0.6096,-0.51435]]

	ORBIT_HORIZONTAL_BOUNDS = [0.25,0.45]
	ACTION_DELAY = 10*1000 #10 seconds

	def __init__(self):
		rospy.init_node("config_to_action",anonymous=True)
		self.current_config = []
		self.add_publisher = rospy.Publisher("add_block", String, queue_size=10)
		self.remove_publisher = rospy.Publisher("remove_block", String, queue_size=10)
		self.home_publisher = rospy.Publisher("jaco_home", String, queue_size=10)
		rospy.Subscriber("/config_targets", String, self.generate_action)
		rospy.Subscriber("/jaco_blocks", String, self.update_current_config)
		rospy.Subscriber("/jaco_moving", String, self.update_jaco_state)
		self.moves_pending = 0
		self.last_action_completed = 0
		rospy.sleep(1)

	def update_current_config(self,message):
		self.current_config = [b for b in eval(message.data)]
		#print("updating config", self.current_config)
		#print(self.blocks2bitstring(self.current_config))

	def get_orbit(self,y):
		for i,orbit in enumerate(self.ORBITS):
			if y > orbit[0] and y < orbit[1]:
				return i
	def blocks2bitstring(self,block_arr):
		raw_btstr = '0'*60
		for block in block_arr:
			if block["x"]< self.ORBIT_HORIZONTAL_BOUNDS[0] or block["x"]>self.ORBIT_HORIZONTAL_BOUNDS[1]:
				continue
			if block["y"]<self.ORBITS[-1][0] or block["y"]>self.ORBITS[0][1]:
				continue
			id = block["id"]
			orbit = self.get_orbit(block["y"])
			index = 12*orbit + id
			raw_btstr = raw_btstr[:index] + '1' + raw_btstr[index+1:] #b/c can't modify string
		return raw_btstr

	def update_jaco_state(self,message):
		if(message.data == "False"):
			self.moves_pending -= 1
			if(self.moves_pending <1):
				self.last_action_completed = message.header.stamp
				self.home_publisher.publish(String())

	def generate_action(self,message):
		if(self.moves_pending >0 or time.time()-self.last_action_completed < self.ACTION_DELAY):
			print("skipping action due to ongoing or too recently completed action")
			return
		print("generating action")
		target_btstr = message.data
		print("target:",target_btstr)
		cur_btstr = self.blocks2bitstring(self.current_config)
		print("current:",cur_btstr)
		if(len(target_btstr) != len(cur_btstr)):
			print("Problem with target bitstring or reading the table. Aborting.")
			return
		action_indices = [i for i,b in enumerate(target_btstr) if b != cur_btstr[i]]
		print(action_indices)
		self.moves_pending = len(action_indices)
		for action in action_indices:
			orbit = action/12
			block = self.BLOCKS[action%12]
			msg = String()
			msg.data = str([block,orbit])
			if target_btstr[action] == '1':
				print("generate add action")
				#add the block
				self.add_publisher.publish(msg)
			else:
				print("generate remove action")
				#remove the block	
				self.remove_publisher.publish(msg)		
			rospy.sleep(0.1)

if __name__ == '__main__':
	try:
		c2a = ConfigToAction()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
