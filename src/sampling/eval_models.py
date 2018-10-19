import numpy as np
import csv
import json
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


# init models
lhc_cost_model = load_model('models/lhc_cost_model.h5')
lhc_science_model = load_model('models/lhc_science_nn_model.h5')
ur_cost_model = load_model('models/ur_cost_model.h5')
ur_science_model = load_model('models/ur_science_nn_model.h5')

ga_cost_model = load_model('models/cost_model_step_60.h5')
ga_science_model = load_model('models/science_model_step_60.h5')

# acquire test data
test_configs, test_cost, test_science = read_data('data/model_test_data.csv')

# code to log residuals for science and cost
lhc_residuals = []
ur_residuals = []
ga_residuals = []

test_configs = np.array(test_configs)
lhc_science_pred = lhc_science_model.predict(test_configs)
lhc_cost_pred = lhc_cost_model.predict(test_configs)
ur_science_pred = ur_science_model.predict(test_configs)
ur_cost_pred = ur_cost_model.predict(test_configs)
ga_science_pred = ga_science_model.predict(test_configs)
ga_cost_pred = ga_cost_model.predict(test_configs)

for i in range(len(test_configs)):
    true_science = test_science[i]
    true_cost = test_cost[i]
    lhc_science_residual = lhc_science_pred[i][0] - true_science
    lhc_cost_residual = lhc_cost_pred[i][0] - true_cost
    ur_science_residual = ur_science_pred[i][0] - true_science
    ur_cost_residual = ur_cost_pred[i][0] - true_cost
    ga_science_residual = ga_science_pred[i][0] - true_science
    ga_cost_residual = ga_cost_pred[i][0] - true_cost
    lhc_residuals.append([lhc_science_residual, lhc_cost_residual])
    ur_residuals.append([ur_science_residual, ur_cost_residual])
    ga_residuals.append([ga_science_residual, ga_cost_residual])

# with open('data/lhc_model_eval.csv', 'w+') as outfile:
#     writer = csv.writer(outfile, delimiter=',')
#     writer.writerow(['science_res', 'cost_res'])
#     for row in lhc_residuals:
#         writer.writerow(row)

# with open('data/ur_model_eval.csv', 'w+') as outfile:
#     writer = csv.writer(outfile, delimiter=',')
#     writer.writerow(['science_res', 'cost_res'])
#     for row in ur_residuals:
#         writer.writerow(row)

with open('data/ga_model_eval.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['science_res', 'cost_res'])
    for row in ga_residuals:
        writer.writerow(row)
