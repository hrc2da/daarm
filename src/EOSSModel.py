from daarm.srv import *
import rospy
import csv


from sklearn import linear_model.LinearRegression as LR

class EOSSModel:
    def __init__(self):
        self.model = LR()
        self.configs = {}
        self.init_evaluate_server()
    def handle_evaluate_config(self,req):
        return this.model.evaluate(req.config)

    def init_evaluate_server(self):
        rospy.init_node('EOSS_model_server')
        s = rospy.Service('EOSS_model_evaluator',EOSSEstimate,handle_evaluate_config)
        rospy.spin()

    def load_data(self,path):
        return None
subscribe and update the model on new evals


