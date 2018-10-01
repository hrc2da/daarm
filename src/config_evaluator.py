#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
import numpy as np
import time

class ConfigEvaluator:

	BLOCKS = ["A","B","C","D","E","F","G","H","I","J","K","L"]

	ORBITS = [[-0.235,-0.14],[-0.3175,-0.235],[-0.4191,-0.3175],[-0.51435,-0.4191],[-0.6096,-0.51435]]

	configs = []

	config_strings = []

	ACTION_DELAY = 0 #the minimum time in sec's between sending new configs
	
	def __init__(self,eval_type="single"):
		rospy.init_node("config_evaluator",anonymous=True)
		self.target_publisher = rospy.Publisher("config_targets", String, queue_size=10)
		if(eval_type=="genetic"):
			self.counter = 0
			self.BATCH_SIZE = 1
			rospy.Subscriber("/populations", String, self.update_pop)
		else:
			rospy.Subscriber("/configs", String, self.eval_config)
		self.last_message_time = time.time()
		rospy.sleep(1)

	def eval_config(self,message):
		#note that this doesn't currently constrain the diffs to one block
		#to do so we would have to discard old configs
		cur_config = json.loads(message.data)
		self.config_strings.append(cur_config["config"])
		self.configs.append([cur_config["cost"],-cur_config["science"]])
		if(time.time()-self.last_message_time < self.ACTION_DELAY):
			print("DELAYING!!!!!!")
			return
		pareto_front = self.is_pareto_efficient_indexed(np.array(self.configs),False)
		print("PARETO FRONT:",pareto_front)
		if(len(self.configs)-1 in pareto_front):
			print("Sending target config")
			#remove the config so we don't build it twice
			self.config_strings.pop(-1)
			self.configs.pop(-1)
			msg = String()
			msg.data = cur_config["config"]
			self.last_message_time = time.time()
			self.target_publisher.publish(msg)
		#for now to get more interactions, grab a pareto dominant config if there is one (may be >1 away)
		elif len(pareto_front)>0:
			config_to_pop = pareto_front[-1]
			target = self.config_strings.pop(config_to_pop)
			self.configs.pop(config_to_pop)
			msg = String()
			msg.data = target
			self.last_message_time = time.time()
			self.target_publisher.publish(msg)

	def update_pop(self,message):
		'''
		update a batch of configs (for gen alg)
		'''
		print("updating pop")
		self.counter += 1
		new_pop = json.loads(message.data)
		self.config_strings.extend([c["config"] for c in new_pop])
		self.configs.extend([[c["cost"],-c["science"]] for c in new_pop])
		
		#truncate the pop
		pareto_indices = self.is_pareto_efficient_indexed(np.array(self.configs),False)
		self.config_strings = [self.config_strings[i] for i in pareto_indices]
		self.configs = [self.configs[i] for i in pareto_indices]
		if self.counter % self.BATCH_SIZE == 0 and time.time()-self.last_message_time > self.ACTION_DELAY:
			print("sending target")
			target = self.config_strings.pop(0)
			self.configs.pop(0)
			msg = String()
			msg.data = target
			self.last_message_time = time.time()
			self.target_publisher.publish(msg)
		
	#stolen from https://stackoverflow.com/questions/32791911/fast-calculation-of-pareto-front-in-python
	def is_pareto_efficient_indexed(self,costs, return_mask = True):  # <- Fastest for many points
	    	"""
    		:param costs: An (n_points, n_costs) array
    		:param return_mask: True to return a mask, False to return integer indices of efficient points.
    		:return: An array of indices of pareto-efficient points.
        		If return_mask is True, this will be an (n_points, ) boolean array
        		Otherwise it will be a (n_efficient_points, ) integer array of indices.
    		"""
    		is_efficient = np.arange(costs.shape[0])
    		n_points = costs.shape[0]
    		next_point_index = 0  # Next index in the is_efficient array to search for

    		while next_point_index<len(costs):
        		nondominated_point_mask = np.any(costs<=costs[next_point_index], axis=1)
        		is_efficient = is_efficient[nondominated_point_mask]  # Remove dominated points
        		costs = costs[nondominated_point_mask]
        		next_point_index = np.sum(nondominated_point_mask[:next_point_index])+1

    		if return_mask:
        		is_efficient_mask = np.zeros(n_points, dtype = bool)
        		is_efficient_mask[is_efficient] = True
        		return is_efficient_mask
    		else:
        		return is_efficient




if __name__ == '__main__':
	try:
		eval_type = rospy.get_param('eval_type', 'single')
		print eval_type
		ce = ConfigEvaluator(eval_type)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
