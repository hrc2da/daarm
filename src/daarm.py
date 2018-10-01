#!/usr/bin/env python

import rospy
import random
import numpy as np
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String

import math

class DAArm:
	'''
	class that controls the DA arm
	subscribes to /blocks
	subscribes to /pick_target
	'''
	BLOCK_HEIGHT = -0.006
	BLOCK_RELEASE_HEIGHT = 0.01
	PRE_GRASP_HEIGHT = 0.05
	ORBIT_HORIZONTAL_BOUNDS = [0.25,0.45]
	BLOCKS = ["A","B","C","D","E","F","G","H","I","J","K","L"]

	ORBITS = [[-0.235,-0.14],[-0.3175,-0.235],[-0.4191,-0.3175],[-0.51435,-0.4191],[-0.6096,-0.51435]]
	MARGIN_THRESHOLD = 0.05 #make this around half the width of a block

	JOINT_NAMES = ['j2s7s300_joint_1', 'j2s7s300_joint_2', 'j2s7s300_joint_3', 'j2s7s300_joint_4', 'j2s7s300_joint_5', 'j2s7s300_joint_6', 'j2s7s300_joint_7', 'j2s7s300_joint_finger_1', 'j2s7s300_joint_finger_2', 'j2s7s300_joint_finger_3']

	SUPPLY = {"A":[0.02,-0.3],"B":[0.12,-0.3],"C":[0.02,-0.36],"D":[0.12,-0.36],"E":[0.02,-0.42],"F":[0.12,-0.42],"G":[0.02,-0.48],"H":[0.12,-0.48],"I":[0.02,-0.54],"J":[0.12,-0.54],"K":[0.02,-0.6],"L":[0.12,-0.6]}
	def __init__(self):
		rospy.init_node("daarm",anonymous=True) #in case we want more than one
		self.moving = False
		self.current_config = []
		self.add_blocks_subscriber = rospy.Subscriber("/add_block", String, self.add_block_wrapper)
		self.remove_blocks_subscriber = rospy.Subscriber("/remove_block", String, self.remove_block_wrapper)
		self.block_update_subscriber = rospy.Subscriber("/jaco_blocks", String, self.update_current_config)
		self.moving_publisher = rospy.Publisher("/jaco_moving", String, queue_size=1)
		self.home_subscriber = rospy.Subscriber("/jaco_home", String, self.home_arm)
		rospy.sleep(1)

		self.scene = PlanningSceneInterface()
		self.robot = RobotCommander()
		self.arm = MoveGroupCommander("arm")
		self.gripper = MoveGroupCommander("gripper")
		self.constraints = Constraints()
		self.j1constraint = JointConstraint()
		self.j1constraint.joint_name = self.JOINT_NAMES[0]
		self.j1constraint.position = 5
		self.j1constraint.tolerance_above = 1
		self.j1constraint.tolerance_below = 1
		self.j2constraint = JointConstraint()
		self.j2constraint.joint_name = self.JOINT_NAMES[1]
		self.j2constraint.position = 3*math.pi/4
		self.j2constraint.tolerance_above = 1
		self.j2constraint.tolerance_below = 1
		self.j4constraint = JointConstraint()
		self.j4constraint.joint_name = self.JOINT_NAMES[3]
		self.j4constraint.position = math.pi/2
		self.j4constraint.tolerance_above = math.pi/4
		self.j4constraint.tolerance_below = math.pi/4
		#self.constraints.joint_constraints.append(self.j2constraint)
		#self.constraints.joint_constraints.append(self.j4constraint)
		self.constraints.joint_constraints.append(self.j1constraint)
		#self.arm.set_path_constraints(self.constraints)
		self.arm.set_num_planning_attempts(10)
		self.arm.set_goal_position_tolerance(0.01)
		print("constraints:",self.arm.get_path_constraints())
		self.pose = self.arm.get_current_pose()
		self.initScene()
		self.arm.set_max_velocity_scaling_factor(1)
		self.arm.set_named_target("Home")
		#plan = self.arm.plan()
		#print(plan.joint_trajectory.points)
		#self.arm.go(wait=True)
		#self.home_arm()
		#self.pick_block(0.55,-0.18)
		#self.place_block(0.55,0.2)
		#rospy.sleep(2)
		#self.pick_block(0.55,0.2)
		#self.place_block(0.55,-0.18)
		
		#self.add_block('D',3)

	#note that current_config contains staging blocks. these are filtered in get_config_block
	def update_current_config(self,message):
                self.current_config = [b for b in eval(message.data)]
                #print("updating config", self.current_config)
                #print(self.blocks2bitstring(self.current_config))

	def get_config_block(self,block,orbit):
		#print("matching",block,orbit)
		#print("curent config",self.current_config)
		#returns a block if it exists, none if not
		if orbit is False:
			#look in the staging area
			matches = [b for b in self.current_config if self.BLOCKS[b['id']]==block \
				and b['x'] <self.ORBIT_HORIZONTAL_BOUNDS[0]] 
		else:
			matches = [b for b in self.current_config if self.BLOCKS[b['id']]==block \
				and b['x'] > self.ORBIT_HORIZONTAL_BOUNDS[0] and b['x'] < self.ORBIT_HORIZONTAL_BOUNDS[1] \
				and b['y'] > self.ORBITS[orbit][0] and b['y'] < self.ORBITS[orbit][1]]
		if len(matches) > 0:
			return matches[0]
		else:
			return None 
	def is_vacant(self,x,y):
		blockers = [b for b in self.current_config if np.linalg.norm([b['x']-x,b['y']-y]) < self.MARGIN_THRESHOLD]
		print("FOUND Blockers: ",len(blockers))
		return len(blockers) < 1

	def add_block_wrapper(self,message):
		if(self.moving):
			return
		self.moving = True
		action = eval(message.data)
		self.add_block(action[0],action[1])
		self.moving = False	
	def remove_block_wrapper(self,message):
		if(self.moving):
			return
		self.moving = True
		action = eval(message.data)
		self.remove_block(action[0],action[1])
		self.moving = False
	def add_block(self,block,orbit):
		#look up the block supply location
		#supply_loc = self.SUPPLY[block]
		msg = String()
		msg.data = "true"
		msg.header.stamp = rospy.Time.now()
		self.moving_publisher.publish(msg)
		pick_block = self.get_config_block(block,False)
		if pick_block is None:
			print("Target block to add is not available in staging area. Aborting.")
			return
		#get a location to put the block in the orbit
		orbit_vert_bounds = self.ORBITS[orbit]
		#for placing blocks in orbits, put it in the center of the orbit for now
		target_y = orbit_vert_bounds[0]+(orbit_vert_bounds[1]-orbit_vert_bounds[0])/2.0
		vacant = False
		while vacant is False:
			target_x = random.uniform(self.ORBIT_HORIZONTAL_BOUNDS[0],self.ORBIT_HORIZONTAL_BOUNDS[1])
			vacant = self.is_vacant(target_x,target_y)
		print("adding block")
		self.pick_block(pick_block["x"],pick_block["y"])
		rospy.sleep(0.5)
		self.place_block(target_x,target_y)
		msg.data = "false"
		msg.header.stamp = rospy.Time.now()
		self.moving_publisher.publish(msg)

	def remove_block(self,block,orbit):
		print("removing block")
		msg = String()
		msg.data = "True"
		msg.header.stamp = rospy.Time.now()
		self.moving_publisher.publish(msg)
		supply_loc = self.SUPPLY[block]
		target_block = self.get_config_block(block,orbit)
		if(target_block is not None):
			self.pick_block(target_block['x'],target_block['y'])
			rospy.sleep(1)
			self.place_block(supply_loc[0],supply_loc[1])
		else:
			print("No matching block found on table!")
		msg.data = "False"
		msg.header.stamp = rospy.Time.now()
		self.moving_publisher.publish(msg)
		return

	def home_arm(self):
		#this doesn't go to the default home position, but one we've found to be better for planning
		print("homing arm")
		p = self.pose
		p.pose.position.x = 0.6
		p.pose.position.y = 0
		p.pose.position.z = 0.6
		#p.pose.orientation = Quaternion(0,1,0,0)
		self.arm.set_pose_target(p)
		self.arm.go(wait=True)
	def set_joint_goals(self,cur_joint_angles,joint_goals):
		#note that the joint goals are assumed to be the first joints in order
		for i,goal in enumerate(joint_goals):
			cur_joint_angles[i] = goal
		return cur_joint_angles 

	def pick_block(self,x,y):
		print("picking")
		
		self.gripper.set_named_target("Open")
		self.gripper.go(wait=True)
		p = self.pose
		p.pose.position.x = x
		p.pose.position.y = y
		p.pose.position.z = self.PRE_GRASP_HEIGHT
		p.pose.orientation = Quaternion(0,1,0.01,0) #gripper oriented down
		self.arm.set_pose_target(p)
		plan = self.arm.plan()
		#joint_goals = plan.joint_trajectory.points[-1].positions
		self.arm.go(wait=True)
		rospy.sleep(0.5)
		#cur_joints = self.arm.get_current_joint_values()
		#print(cur_joints)
		#print(joint_goals)
		#joint_targets = self.set_joint_goals(cur_joints,joint_goals)
		print("correcting pose")
		#self.arm.go(joint_targets, wait=True)
		#self.arm.stop()
		self.arm.set_pose_target(p)
		self.arm.go()
		rospy.sleep(1)
		print("descending")
		p.pose.position.z = self.BLOCK_HEIGHT
		self.arm.set_pose_target(p)
		self.arm.go()
		self.gripper.set_named_target("Close")
		self.remove_tui()
		self.gripper.go(wait=True)
		self.add_tui()
		p.pose.position.z = self.PRE_GRASP_HEIGHT
		self.arm.set_pose_target(p)
		self.arm.go()
	def place_block(self,x,y):	
		print("placing")
		p = self.pose
		p.pose.position.x = x
		p.pose.position.y = y
		p.pose.position.z = self.PRE_GRASP_HEIGHT
		p.pose.orientation = Quaternion(0,1,0,0)
		self.arm.set_pose_target(p)
		self.arm.go(wait=True)
		rospy.sleep(0.5)
		self.arm.set_pose_target(p)
		self.arm.go(wait=True)
		rospy.sleep(0.5)
		p.pose.position.z = self.BLOCK_RELEASE_HEIGHT
		self.arm.set_pose_target(p)
		self.arm.go(wait=True)
		self.gripper.set_named_target("Open")
		self.remove_tui()
		self.gripper.go(wait=True)
		self.add_tui()
		p.pose.position.z = self.PRE_GRASP_HEIGHT
		self.arm.set_pose_target(p)
		self.arm.go()
		print(p)
	def add_tui(self):
		self.scene.add_box("tui", self.tuiPose, self.tuiDimension)
		rospy.sleep(1)
	def remove_tui(self):
		self.scene.remove_world_object("tui")
		rospy.sleep(1)
	def initScene(self):
		#todo: compress this to a function and a dictionary
		self.scene.remove_world_object("table")
		self.scene.remove_world_object("roman")
		self.scene.remove_world_object("tui")
		self.scene.remove_world_object("monitor")
		self.scene.remove_world_object("overhead")
		self.scene.remove_world_object("wall")
		rospy.sleep(2)

		#initialize poses for some collision objects
		self.tablePose = PoseStamped()
		self.tablePose.header.frame_id = self.robot.get_planning_frame() 
		self.romanPose = PoseStamped()
		self.romanPose.header.frame_id = self.robot.get_planning_frame()
		self.monitorPose = PoseStamped()
		self.monitorPose.header.frame_id = self.robot.get_planning_frame()
		self.tuiPose = PoseStamped()
		self.tuiPose.header.frame_id = self.robot.get_planning_frame()
		self.wallPose = PoseStamped()
		self.wallPose.header.frame_id = self.robot.get_planning_frame()

		#define where each object is and their dimensions
		#self.tablePose.pose.position = Point(-0.6096/2+0.1,0.1016,-0.7239/2) #0.1016 is from measurement that the arm is that much off center
		#self.tableDimensions = (0.6096,1.524,0.7239)
		#self.romanPose.pose.position = Point(-0.6096/2+0.1,0.1016-1.524/2+0.41/2,0.58/2)
		#self.romanDimensions = (0.6095,0.41,0.58)
		#self.tuiPose.pose.position = Point(0.1+0.12+0.9906/2,0.1016+1.524/2-(0.255+0.8382/2),-0.8255/2+0.085) #(arm_base_width+gap+table_width,
		#self.tuiDimensions = (0.9906,0.8382,0.8255)
		#self.monitorPose.pose.position = Point(self.tuiPose.pose.position.x,self.tuiPose.pose.position.y+self.tuiDimensions[1]/2-0.2667/2,self.tuiPose.pose.position.z+self.tuiDimensions[2]/2+0.5588/2)
		#self.monitorDimensions = (0.5588,0.2667,0.5588)
		#add the objects to the scene
		#rospy.sleep(2)
		#self.scene.add_box("table", self.tablePose, self.tableDimensions)
		#self.scene.add_box("roman", self.romanPose, self.romanDimensions)
		#self.scene.add_box("tui", self.tuiPose, self.tuiDimensions)
		#self.scene.add_box("monitor", self.monitorPose, self.monitorDimensions)
		
		#top of the table safety constraints:
		self.tuiPose.pose.position = Point(0.3556,-0.343,-0.51)
		self.tuiDimension = (0.9906, 0.8382, 0.8636)
		self.wallPose.pose.position = Point(-0.508, -0.343 ,-0.3048)
		self.wallDimension = (0.6096, 2 , 1.35)
		rospy.sleep(2)
		self.scene.add_box("tui", self.tuiPose, self.tuiDimension)
		self.scene.add_box("wall", self.wallPose, self.wallDimension)


if __name__ == '__main__':
	arm = DAArm()
	rospy.spin()
