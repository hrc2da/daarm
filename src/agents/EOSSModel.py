#from daarm.srv import *
import rospy
import csv
from std_msgs.msg import String

import numpy as np
import pickle as pk
from sklearn.linear_model import LinearRegression as LR
from sklearn.model_selection import train_test_split
from sklearn.externals import joblib
from sklearn.svm import SVR
from keras.models import Sequential, load_model
from keras.layers import Dense
from keras.layers import Dropout
from keras import backend as K
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import train_test_split
import json

from science_model import build_model as bm
from science_model import listify_config


class EOSSModel:
    def __init__(self, predata_path=""):
        self.tf_session =  K.get_session()
        #self.tf_graph = tf.get_default_graph()
        self.cost_model = self.build_science_model() #we are using an mlp for both
        self.configs = {}
        self.batch_size = 10
        self.configs['0' * 60] = (0, 0)
        self.science_model = self.build_science_model()
        if(predata_path):
            self.load_data(predata_path)
            self.train_model()
        #self.initevaluate_server()
        rospy.Subscriber("/configs", String, self.add_training_point)

    def add_training_point(self, ros_msg):
        point = json.loads(ros_msg.data)
        self.configs[point['config']] = (point['science'], point['cost'])
        if len(self.configs) % self.batch_size == 0:
            self.train_model()

    def handle_evaluate_config(self, req):
        config = np.array(self.conv_bit_list(req['config'])).reshape(1, -1)
        science = self.science_model.predict(config)[0][0]
        cost = self.cost_model.predict(config)[0][0]
        #print(science)
        #print(cost)
        return [science, cost]

    @staticmethod
    def conv_bit_list(bitstring):
        return np.array([int(bit) for bit in bitstring])

    def build_science_model(self):
        #model = load_model('/home/dev/catkin_ws_kinova/src/daarm/modelscience_nn_model.h5')
        #return model
        #science_regressor = KerasRegressor(
        #    build_fn=bm, epochs=20, batch_size=32, verbose=1)
        #return science_regressor.model
        model = self.bm()
        XTr_s = np.array([self.conv_bit_list('0'*60)])
        yTr_s = np.array([0.0])
        model.fit(XTr_s,yTr_s)
        return model

    @staticmethod
    def bm():
        model = Sequential()
        model.add(Dense(1024, input_dim=60,
                        kernel_initializer='normal', activation='relu'))
        model.add(Dropout(0.2))
        model.add(Dense(512, kernel_initializer='normal', activation='relu'))
        model.add(Dropout(0.2))
        model.add(Dense(512, kernel_initializer='normal', activation='relu'))
        model.add(Dropout(0.2))
        model.add(Dense(1, kernel_initializer='normal'))
        model.compile(loss='mse', optimizer='adam')
        return model

    def build_model(self):
        model = load_model('/home/dev/catkin_ws_kinova/src/daarm/src/agents/science_nn_model.h5')
        return model

    def build_cost_model(self):
        #with open("/home/dev/catkin_ws_kinova/src/daarm/model/cost_model.pk", 'rb') as pk_file:
        #    model = pk.load(pk_file).cost_model #this is ridiculous. we pickled the wrong thing.
        #return model
        model = LR()
        XTr_s = np.array([self.conv_bit_list('0'*60)])
        yTr_s = np.array([0.0])
        model.fit(XTr_s,yTr_s)
        return model

    def train_model(self):
        X = np.array([self.conv_bit_list(config)
                      for config in self.configs.keys()])
        science,cost = map(np.array, zip(*self.configs.values()))
        print(X, science, cost)
        #self.cost_model.fit(X, cost)
        self.cost_model.fit(X, cost, epochs=20)
        self.science_model.fit(X, science, epochs=20)
        self.cost_model.save("/home/dev/.ros/dalogs/cost_model_step_"+str(len(self.configs))+".h5")
        self.science_model.save("/home/dev/.ros/dalogs/science_model_step_"+str(len(self.configs))+".h5")
        
        #joblib.dump(self.cost_model, "/home/dev/.ros/dalogs/cost_model_step_"+str(len(self.configs))+".pk")
        #XTr_s = np.array([listify_config('0'*60)])
        #yTr_s = np.array([0.0])
        #self.science_model.fit(XTr_s, yTr_s, epoch=20)
        #xTr, xTe, yTr, yTe = train_test_split(X, Y, shuffle = True)
        # self.model.fit(xTr,yTr)

    def init_evaluate_server(self):
        rospy.init_node('EOSS_model_server')
        s = rospy.Service('EOSS_model_evaluator',
                          EOSSEstimate, self.handle_evaluate_config)
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
    #m = EOSSModel("/home/nikhildhawan/catkin_ws_kinova/src/daarm/src/model/raw_combined_data.csv")
    m = EOSSModel()
