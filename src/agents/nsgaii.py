#! /usr/bin/env python
import sys
import traceback
import random
import json
import numpy as np
from functools import reduce
import requests
import pickle as pk
from Queue import Queue, Empty
from platypus.operators import TournamentSelector, RandomGenerator
#from schwimmbad import MultiPool
from platypus import NSGAII, Problem, Type, Real, Binary, Integer, ProcessPoolEvaluator, PoolEvaluator, CompoundOperator, SBX, HUX, MultiprocessingEvaluator, run_job, Generator
from platypus.config import default_variator
from platypus.core import nondominated_sort, nondominated_truncate, Solution
from EOSSModel import EOSSModel
from EOSSSingleModel import EOSSSingleModel
import rospy
from std_msgs.msg import String
#from daarm.srv import *
import time
import logging
import requests
import redis



class DA_NSGAII(NSGAII):
    PUBLISH_BATCH = 100
    def __init__(self, problem, population_size=100, generator=RandomGenerator(), selector=TournamentSelector(2),
                 variator=None, archive=None, injection_probability = 1, **kwargs):
        super(DA_NSGAII, self).__init__(problem, population_size, generator,
                                        selector, variator, archive, **kwargs)
        self.logger = logging.getLogger()
        self.logger = logging.getLogger("nsgaii logger")
        filehandler = logging.FileHandler('./dalogs/nsgaii.log')
        self.logger.setLevel(logging.DEBUG)
        self.logger.addHandler(filehandler)
        #self.logger.basicConfig(filename='./dalogs/nsgaii.log',level=logging.DEBUG)
        rospy.init_node('da_nsgaii')
        self.population_publisher = rospy.Publisher('/populations',String, queue_size=10)
        self.generation_count = 0
        self.injection_probability = injection_probability
        self.observed_configs = Queue()
        self.config_listener = rospy.Subscriber(
            '/configs', String, self.observe_new_configs)


    def observe_new_configs(self, message):
        config = json.loads(message.data)["config"]
        solution_config = self.bitstring2solution(config)
        self.observed_configs.put(solution_config)

    def bitstring2solution(self, bitstring):
        solution = Solution(self.problem)
        solution.variables.__setitem__(0, [bool(int(bit)) for bit in bitstring]) 
        return solution

    def choose_observed_configs(self):
        chosen_configs = []
        num_observed_configs = self.observed_configs.qsize()
        
        for _ in range(num_observed_configs):
            try:
                config = self.observed_configs.get()
                if (np.random.uniform(0,1) <= self.injection_probability):
                    chosen_configs.append(config)
            except Empty:
                continue
        return chosen_configs

    def iterate(self):
        offspring = []
        self.generation_count +=1
        while len(offspring) < self.population_size:
            parents = self.selector.select(self.variator.arity, self.population)
            offspring.extend(self.variator.evolve(parents))
        #print("before injection: ",len(offspring))
        chosen_configs = self.choose_observed_configs()
        #print("chosen_configs: ", len(chosen_configs))
        offspring.extend(chosen_configs)
        #print("after injection: ",len(offspring))
        
        offspring.extend(self.population)
        for o in offspring:
            o.evaluated = False
        self.evaluate_all(offspring)
        nondominated_sort(offspring)
        self.population = nondominated_truncate(offspring, self.population_size)
        logtime = time.time()
        publish_set = []
        for sol in self.population:
            config = ''.join([str(int(x)) for x in sol.variables[0]])
            science,cost = sol.objectives
            self.logger.info("{},{},{},{},{}".format(self.generation_count,logtime,config,science,cost))
            publish_set.append({"config":config,"cost":float(cost),"science":float(science)})
        if(self.generation_count % self.PUBLISH_BATCH == 0):
            #print("publish set",publish_set)
            msg = String()
            msg.data = json.dumps(publish_set)
            self.population_publisher.publish(msg)
        if self.archive is not None:
            self.archive.extend(self.population)






'''
Input:
60 bit binary one-hot
Objectives:
science
cost
'''


class nsgaii_agent:
    def __init__(self, session_id=None, model=None, use_surrogate=True):
        print "INITIALIZING"
        self.EVAL_URI = "https://www.selva-research.com/api/vassar/evaluate-architecture"
        self.SESSION_URI = "https://www.selva-research.com/api/daphne/set-problem"
        self.n_iters = 1000000
        self.model = model
        self.session_id = session_id
        self.problem = Problem(1, 2)
        self.problem.types = [Binary(60)]
        self.problem.function = self.evaluate
        self.problem.directions = [
            self.problem.MAXIMIZE, self.problem.MINIMIZE]
        self.vassar_session = requests.session()
        self.vassar_session.post(self.SESSION_URI,json={"problem":"ClimateCentric"})
        self.cached_evals = redis.Redis(host="localhost",port=6379) #defaults
        self.use_surrogate = use_surrogate

    def evaluate_REST(self,config):
        result = self.cached_evals.get(config)
        if result is None:
            packed_config = str([bool(int(x)) for x in config]).lower()
            print("retreiving point")
            response = self.vassar_session.post(self.EVAL_URI,json={"special":"False","inputs":packed_config})
            print("retrieved point")
            result = str(response.json()["outputs"])[1:-1] #weird thing to make the result a comma-sep string for consistency
            self.cached_evals.set(config,result)
        science,cost = map(float,result.split(','))
        return science, cost
        

    def evaluate(self, config):
        # convert config to bitstring
        # call EOSSModel service
        # rospy.wait_for_service('EOSS_model_evaluator')
        #result = rospy.ServiceProxy('EOSS_model_evaluator', EOSSEstimate)
        #print("EVALUATING CONFIG WITH UPDATED MODEL")
        if (not self.use_surrogate):
            print("eval point")
            science, cost = self.evaluate_REST(config[0])
            return [science, cost]
        if(self.model):
            msg = {"config": "".join([str(int(x)) for x in config[0]])}
           # print("current cost model",self.model.cost_model.coef_)
            result = self.model.handle_evaluate_config(msg)
            return result

    def run(self):
        algorithm = DA_NSGAII(self.problem, population_size=50, injection_probability=1)
        algorithm.run(self.n_iters)
        print "Saving EOSSModel"
        curr_time = str(time.time())
        path = "/home/dev/.ros/dalogs/"
        self.model.science_model.save(path+"science_model"+curr_time+".h5")
        with open(path+"cost_model"+curr_time+".pk", "wb") as pk_file:
            pk.dump(self.model.cost_model, pk_file)
        #s = requests.Session()
        #s.post("https://www.selva-research.com/api/daphne/set-problem",
        #       json={"problem": "ClimateCentric"})
        for sol in algorithm.result:
            print(sol.objectives)
            #result = s.post("https://www.selva-research.com/api/vassar/evaluate-architecture",
            #                json={"special": "False", "inputs": str(sol.variables[0]).lower()})
            #print(result.json()['outputs'])


if __name__ == "__main__":
    e = EOSSModel()
    #e = EOSSModel("/home/dev/catkin_ws_kinova/src/daarm/model/raw_combined_data.csv")
    agent = nsgaii_agent(model=e, use_surrogate = False)
    try:
        agent.run()
    except Exception as e:
        traceback.print_exc(file=sys.stdout)
        print("problem", e)
