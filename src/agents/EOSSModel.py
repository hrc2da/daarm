#from daarm.srv import *
#import rospy
import csv

import numpy as np
from sklearn.linear_model import LinearRegression as LR
from sklearn.model_selection import train_test_split
from sklearn.svm import SVR
from keras.models import Sequential, load_model
from keras.layers import Dense
from keras.layers import Dropout
from keras import backend as K
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import train_test_split
import json


class EOSSModel:
    def __init__(self, predata_path=""):
        self.model = LR()
        self.configs = {}
        self.batch_size = 1
        self.configs['0' * 60] = (0, 0)
        self.science_model = self.build_model()
        if(predata_path):
            self.load_data(predata_path)
        self.train_model()
        # self.initevaluate_server()
        #rospy.Subscriber("/configs", String, self.add_training_point)

    def add_training_point(self, ros_msg):
        point = json.loads(ros_msg.data)
        self.configs[point['config']] = (point['science'], point['cost'])
        if len(self.configs) % self.batch_size == 0:
            self.train_model()

    def handle_evaluate_config(self, req):
        config = np.array(self.conv_bit_list(req['config'])).reshape(1, -1)
        return [self.science_model.predict(config)[0][0], self.model.predict(config)[0][1]]

    @staticmethod
    def conv_bit_list(bitstring):
        return np.array([int(bit) for bit in bitstring])

    def build_model(self):
        model = load_model('./src/agents/science_nn_model.h5')
        return model

    def train_model(self):
        X = np.array([self.conv_bit_list(config)
                      for config in self.configs.keys()])
        Y = np.array(self.configs.values())
        self.model.fit(X, Y)
        #xTr, xTe, yTr, yTe = train_test_split(X, Y, shuffle = True)
        # self.model.fit(xTr,yTr)

    def init_evaluate_server(self):
        rospy.init_node('EOSS_model_server')
        s = rospy.Service('EOSS_model_evaluator',
                          EOSSEstimate, handle_evaluate_config)
        rospy.spin()

    def load_data(self, path):
        try:
            with open(path, 'rb') as predata:
                csvreader = csv.reader(predata, delimiter=',')
                next(csvreader)
                for row in csvreader:
                    self.configs[row[0]] = (float(row[1]), float(
                        row[2]))  # config = science,cost
        except IOError as e:
            print(e)
            return


if __name__ == '__main__':
    m = EOSSModel("model/raw_combined_data.csv")
