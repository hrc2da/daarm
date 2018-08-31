import sys
import traceback
import random
import json
import numpy as np
from functools import reduce
import requests
from Queue import Queue, Empty
from platypus.operators import TournamentSelector, RandomGenerator
#from schwimmbad import MultiPool
from platypus import NSGAII, Problem, Type, Real, Binary, Integer, ProcessPoolEvaluator, PoolEvaluator, CompoundOperator, SBX, HUX, MultiprocessingEvaluator, run_job, Generator
from platypus.config import default_variator
from platypus.core import nondominated_sort, nondominated_truncate, Solution
from EOSSModel import EOSSModel
#import rospy
#from daarm.srv import *


class DA_NSGAII(NSGAII):
    def __init__(self, problem, population_size=100, generator=RandomGenerator(), selector=TournamentSelector(2),
                 variator=None, archive=None, injection_probability = 0.5, **kwargs):
        super(DA_NSGAII, self).__init__(self, problem, population_size, generator,
                                        selector, variator, archive, **kwargs)
        rospy.init_node('da_nsgaii')
        self.injection_probability = injection_probability
        self.observed_configs = Queue()
        self.config_listener = rospy.Subscriber(
            '/configs', self.observe_new_configs)

    def observe_new_configs(self, message):
        config = json.loads(message)["config"]
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
        
        while len(offspring) < self.population_size:
            parents = self.selector.select(self.variator.arity, self.population)
            offspring.extend(self.variator.evolve(parents))
        offspring.extend(self.choose_observed_configs())
        self.evaluate_all(offspring)
        
        offspring.extend(self.population)
        nondominated_sort(offspring)
        self.population = nondominated_truncate(offspring, self.population_size)
        
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
    def __init__(self, session_id=None, model=None):
        self.n_iters = 1000
        self.model = model
        self.session_id = session_id
        self.problem = Problem(1, 2)
        self.problem.types = [Binary(60)]
        self.problem.function = self.evaluate
        self.problem.directions = [
            self.problem.MAXIMIZE, self.problem.MINIMIZE]

    def evaluate(self, config):
        # convert config to bitstring
        # call EOSSModel service
        # rospy.wait_for_service('EOSS_model_evaluator')
        #result = rospy.ServiceProxy('EOSS_model_evaluator', EOSSEstimate)
        if(self.model):
            msg = {"config": "".join([str(int(x)) for x in config[0]])}
            result = self.model.handle_evaluate_config(msg)
            return result

    def run(self):
        algorithm = DA_NSGAII(self.problem, population_size=50)
        algorithm.run(self.n_iters)
        s = requests.Session()
        s.post("https://www.selva-research.com/api/daphne/set-problem",
               json={"problem": "ClimateCentric"})
        for sol in algorithm.result:
            print(sol.objectives)
            result = s.post("https://www.selva-research.com/api/vassar/evaluate-architecture",
                            json={"special": "False", "inputs": str(sol.variables[0]).lower()})
            print(result.json()['outputs'])


if __name__ == "__main__":
    e = EOSSModel("./model/raw_combined_data.csv")
    agent = nsgaii_agent(model=e)
    try:
        agent.run()
    except Exception as e:
        traceback.print_exc(file=sys.stdout)
        print("problem", e)
