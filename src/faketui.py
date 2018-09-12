#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests
#import Queue.Queue
from random import shuffle, random, randint
import redis
import time
import json
import time

import logging
logging.basicConfig(filename='./dalogs/faketui.log',level=logging.DEBUG)

class FakeTUI:
    '''
    The FakeTUI does three primary things:
    (1) listen for configs to put on the table and evaluate on /target_configs
    (2) generate new configs to evaluate using basic LSA
    (3) publish all evaluated configs on /configs
    '''
    BUILD_DELAY = 0 #if we want to insert a delay to simulate how long it takes to build a config
    EVAL_DELAY = 1
    EVAL_URI = "https://www.selva-research.com/api/vassar/evaluate-architecture"
    SESSION_URI = "https://www.selva-research.com/api/daphne/set-problem"
    def __init__(self,num_bits=60):
        rospy.init_node("fake_tui",anonymous=True)
        self.cur_config = '0'*num_bits
        #self.to_evalute = Queue()
        self.config_listener = rospy.Subscriber('config_targets', String, self.handle_new_config)
        self.eval_publisher = rospy.Publisher('configs', String, queue_size=1)
        self.cached_evals = redis.Redis(host="localhost",port=6379) #defaults
        self.vassar_session = requests.session()
        self.vassar_session.post(self.SESSION_URI,json={"problem":"ClimateCentric"})
        self.generated_configs = []
        while not rospy.core.is_shutdown():
            config = self.generate_config()
            if config not in self.generated_configs:
                rospy.sleep(5)
                self.generated_configs.append(config)
                self.evaluate_REST(config,"local")
            rospy.rostime.wallsleep(0.5)
    def handle_new_config(self,msg):
        #we could just overwrite self.cur_config but instead are going to flip bits one at a time to simulate the arm building the config
        target_config = msg.data
        print("targeting "+target_config)
        print("from "+self.cur_config)
        add_bits,rm_bits = self.config_to_actions(target_config)
        print(len(add_bits),len(rm_bits))
        #update the table in some random order of changes
        shuffle(add_bits)
        shuffle(rm_bits) 
        while len(add_bits) + len(rm_bits) > 0:
            rospy.sleep(self.BUILD_DELAY)
            if len(add_bits) > 0 and len(rm_bits) > 0:
                if random() > 0.5:
                    idx = add_bits.pop()
                    self.cur_config = self.cur_config[:idx]+'1'+self.cur_config[idx+1:]
                else:
                    idx = rm_bits.pop()
                    self.cur_config = self.cur_config[:idx]+'0'+self.cur_config[idx+1:]
            elif len(add_bits) > 0:
                idx = add_bits.pop()
                self.cur_config = self.cur_config[:idx]+'1'+self.cur_config[idx+1:]
            else:
                idx = rm_bits.pop()
                self.cur_config = self.cur_config[:idx]+'0'+self.cur_config[idx+1:]
            
            self.evaluate_REST(self.cur_config,"table") 
    def config_to_actions(self,target_config):
        #compares a target to cur_config and then generates of tuple of indices to add and indices to remove (add,remove)
        cur_config = self.cur_config
        add_bits = [i for i,b in enumerate(target_config) if b == '1' and cur_config[i] == '0']
        rm_bits = [i for i,b in enumerate(target_config) if b == '0' and cur_config[i] == '1']
        return (add_bits,rm_bits)
    def evaluate_REST(self,config,source="table"):
        result = self.cached_evals.get(config)
        if result is None:
            packed_config = str([bool(int(x)) for x in config]).lower()
            response = self.vassar_session.post(self.EVAL_URI,json={"special":"False","inputs":packed_config})
            result = str(response.json()["outputs"])[1:-1] #weird thing to make the result a comma-sep string for consistency
            self.cached_evals.set(config,result)
        else:
            rospy.sleep(self.EVAL_DELAY)
        science,cost = map(float,result.split(','))
        msg = String()
        msg.data = json.dumps({"config":config, "science":science, "cost":cost})
        logstr = "{},{},{},{},{}".format(source,config,science,cost,time.time())
        print(logstr)
        logging.info(logstr)
        self.eval_publisher.publish(msg)

    def generate_config(self):
        bit_to_flip = randint(0,len(self.cur_config)-1)
        flipped_val = str(1 ^ int(self.cur_config[bit_to_flip]))
        new_config = self.cur_config[:bit_to_flip]+flipped_val+self.cur_config[bit_to_flip+1:]
        return new_config
    
if __name__=='__main__':
    ft = FakeTUI()