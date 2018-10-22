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
lhc_cost_model = load_model('report/models/lhc_cost_model.h5')
lhc_science_model = load_model('report/models/lhc_science_model.h5')
ur_cost_model = load_model('report/models/ur_cost_model.h5')
ur_science_model = load_model('report/models/ur_science_model.h5')
t1_cost_model = load_model('report/models/t1_cost_model.h5')
t1_science_model = load_model('report/models/t1_science_model.h5')
t2_cost_model = load_model('report/models/t2_cost_model.h5')
t2_science_model = load_model('report/models/t2_science_model.h5')
t3_cost_model = load_model('report/models/t3_cost_model.h5')
t3_science_model = load_model('report/models/t3_science_model.h5')
expert_cost_model = load_model('report/models/expert_cost_model.h5')
expert_science_model = load_model('report/models/expert_science_model.h5')

# acquire test data
test_configs, test_cost, test_science = read_data('report/data/model_test_data.csv')
test_configs = np.array(test_configs)

# code to log residuals for science and cost
lhc_residuals = []
ur_residuals = []
t1_residuals = []
t2_residuals = []
t3_residuals = []
expert_residuals = []

lhc_science_pred = lhc_science_model.predict(test_configs)
lhc_cost_pred = lhc_cost_model.predict(test_configs)
ur_science_pred = ur_science_model.predict(test_configs)
ur_cost_pred = ur_cost_model.predict(test_configs)
t1_science_pred = t1_science_model.predict(test_configs)
t1_cost_pred = t1_cost_model.predict(test_configs)
t2_science_pred = t2_science_model.predict(test_configs)
t2_cost_pred = t2_cost_model.predict(test_configs)
t3_science_pred = t3_science_model.predict(test_configs)
t3_cost_pred = t3_cost_model.predict(test_configs)
expert_science_pred = expert_science_model.predict(test_configs)
expert_cost_pred = expert_cost_model.predict(test_configs)

for i in range(len(test_configs)):
    true_science = test_science[i]
    true_cost = test_cost[i]
    lhc_science_residual = lhc_science_pred[i][0] - true_science
    lhc_cost_residual = lhc_cost_pred[i][0] - true_cost
    ur_science_residual = ur_science_pred[i][0] - true_science
    ur_cost_residual = ur_cost_pred[i][0] - true_cost
    t1_science_residual = t1_science_pred[i][0] - true_science
    t1_cost_residual = t1_cost_pred[i][0] - true_cost
    t2_science_residual = t2_science_pred[i][0] - true_science
    t2_cost_residual = t2_cost_pred[i][0] - true_cost
    t3_science_residual = t3_science_pred[i][0] - true_science
    t3_cost_residual = t3_cost_pred[i][0] - true_cost
    expert_science_residual = expert_science_pred[i][0] - true_science
    expert_cost_residual = expert_cost_pred[i][0] - true_cost
    lhc_residuals.append([lhc_science_residual, lhc_cost_residual])
    ur_residuals.append([ur_science_residual, ur_cost_residual])
    t1_residuals.append([t1_science_residual, t1_cost_residual])
    t2_residuals.append([t2_science_residual, t2_cost_residual])
    t3_residuals.append([t3_science_residual, t3_cost_residual])
    expert_residuals.append([expert_science_residual, expert_cost_residual])

with open('report/data/residuals/lhc_model_eval.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['science_res', 'cost_res'])
    for row in lhc_residuals:
        writer.writerow(row)

with open('report/data/residuals/ur_model_eval.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['science_res', 'cost_res'])
    for row in ur_residuals:
        writer.writerow(row)

with open('report/data/residuals/t1_model_eval.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['science_res', 'cost_res'])
    for row in t1_residuals:
        writer.writerow(row)

with open('report/data/residuals/t2_model_eval.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['science_res', 'cost_res'])
    for row in t2_residuals:
        writer.writerow(row)

with open('report/data/residuals/t3_model_eval.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['science_res', 'cost_res'])
    for row in t3_residuals:
        writer.writerow(row)

with open('report/data/residuals/expert_model_eval.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['science_res', 'cost_res'])
    for row in expert_residuals:
        writer.writerow(row)
