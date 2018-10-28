import numpy as np
import csv
import os
import json
from collections import defaultdict
from keras.models import Sequential, load_model
from keras.layers import Dense
from keras.layers import Dropout
from keras import backend as K
from keras.wrappers.scikit_learn import KerasRegressor


def listify_config(config):
    l_config = []
    for bit in config:
        l_config.append((int(bit)))
    return l_config


def read_data(path):
    with open(path, 'r+') as raw_data:
        reader = csv.reader(raw_data, delimiter=',')
        next(reader)
        configs = []
        cost = []
        science = []
        for row in reader:
            configs.append(listify_config(row[0]))
            cost.append(float(row[2]))
            science.append(float(row[1]))
    return configs, cost, science


directory = 'report/models/user_models/'
all_files = os.listdir(directory)

print("importing models")
i = 0
test_configs, test_cost, test_science = read_data('report/data/model_validation_data.csv')
test_configs = np.array(test_configs)
test_cost = np.array(test_cost)
test_science = np.array(test_science)

for file in all_files:
    i += 1
    print(str(i))
    model = load_model(directory + file)
    if 'science' in file:
        predictions = model.predict(test_configs)
        curr_residuals = predictions.flatten() - test_science.flatten()
        if 't1' in file:
            category = 't1_science_residuals'
            with open('report/data/residuals/all_user/'+category+'.csv', 'a+') as outfile:
                writer = csv.writer(outfile, delimiter=',')
                for res_val in curr_residuals:
                    writer.writerow([res_val])
        elif 't2' in file:
            category = 't2_science_residuals'
            with open('report/data/residuals/all_user/'+category+'.csv', 'a+') as outfile:
                writer = csv.writer(outfile, delimiter=',')
                for res_val in curr_residuals:
                    writer.writerow([res_val])
        elif 't3' in file:
            category = 't3_science_residuals'
            with open('report/data/residuals/all_user/'+category+'.csv', 'a+') as outfile:
                writer = csv.writer(outfile, delimiter=',')
                for res_val in curr_residuals:
                    writer.writerow([res_val])
    else:
        predictions = model.predict(test_configs)
        curr_residuals = predictions.flatten() - test_cost.flatten()
        if 't1' in file:
            category = 't1_cost_residuals'
            with open('report/data/residuals/all_user/'+category+'.csv', 'a+') as outfile:
                writer = csv.writer(outfile, delimiter=',')
                for res_val in curr_residuals:
                    writer.writerow([res_val])
        elif 't2' in file:
            category = 't2_cost_residuals'
            with open('report/data/residuals/all_user/'+category+'.csv', 'a+') as outfile:
                writer = csv.writer(outfile, delimiter=',')
                for res_val in curr_residuals:
                    writer.writerow([res_val])
        elif 't3' in file:
            category = 't3_cost_residuals'
            with open('report/data/residuals/all_user/'+category+'.csv', 'a+') as outfile:
                writer = csv.writer(outfile, delimiter=',')
                for res_val in curr_residuals:
                    writer.writerow([res_val])
