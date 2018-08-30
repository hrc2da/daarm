import sys, traceback
import random
from functools import reduce
import requests
#import rospy
#from daarm.srv import *

#from schwimmbad import MultiPool
from platypus import NSGAII, Problem, Type, Real, Binary, Integer,ProcessPoolEvaluator, PoolEvaluator,CompoundOperator,SBX,HUX,MultiprocessingEvaluator, run_job, Generator
from platypus.config import default_variator
from  EOSSModel import EOSSModel
'''
Input:
60 bit binary one-hot
Objectives:
science
cost
'''
class nsgaii_agent:
        def __init__(self,session_id=None,model=None):
                self.n_iters=1000
                self.model = model
                self.session_id=session_id
                self.problem = Problem(1,2)
                self.problem.types = [Binary(60)]
                self.problem.function = self.evaluate
                self.problem.directions = [self.problem.MAXIMIZE,self.problem.MINIMIZE]
        def evaluate(self, config):
            #convert config to bitstring
            #call EOSSModel service
            #rospy.wait_for_service('EOSS_model_evaluator')
            #result = rospy.ServiceProxy('EOSS_model_evaluator', EOSSEstimate)
            if(self.model):
                msg = {"config": "".join([str(int(x)) for x in config[0]])}
                result = self.model.handle_evaluate_config(msg)
                return result

        def run(self):
            algorithm = NSGAII(self.problem, population_size=50)
            algorithm.run(self.n_iters)
            s = requests.Session()
            s.post("https://www.selva-research.com/api/daphne/set-problem",json={"problem":"ClimateCentric"})
            for sol in algorithm.result:
                print(sol.objectives)
                result = s.post("https://www.selva-research.com/api/vassar/evaluate-architecture",json={"special":"False","inputs":str(sol.variables[0]).lower()})
                print(result.json()['outputs'])

if __name__ == "__main__":
    e = EOSSModel("./model/raw_combined_data.csv")
    agent = nsgaii_agent(model = e)
    try:
        agent.run()
    except Exception as e:
        traceback.print_exc(file=sys.stdout)    
        print("problem", e)