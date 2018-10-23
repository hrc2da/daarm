import matplotlib.pyplot as plt
import numpy as np
import csv
import seaborn as sns

lhc_science_residuals = []
lhc_cost_residuals = []
ur_science_residuals = []
ur_cost_residuals = []
t1_science_residuals = []
t1_cost_residuals = []
t2_science_residuals = []
t2_cost_residuals = []
t3_science_residuals = []
t3_cost_residuals = []
limit_science_residuals = []
limit_cost_residuals = []

with open('report/data/residuals/set_2/lhc_model_eval.csv', 'r+') as lhc_in:
    reader = csv.reader(lhc_in, delimiter=',')
    next(reader)
    for row in reader:
        lhc_science_residuals.append(float(row[0]))
        lhc_cost_residuals.append(float(row[1]))

with open('report/data/residuals/set_2/ur_model_eval.csv', 'r+') as ur_in:
    reader = csv.reader(ur_in, delimiter=',')
    next(reader)
    for row in reader:
        ur_science_residuals.append(float(row[0]))
        ur_cost_residuals.append(float(row[1]))

with open('report/data/residuals/set_2/t1_model_eval.csv', 'r+') as t1_in:
    reader = csv.reader(t1_in, delimiter=',')
    next(reader)
    for row in reader:
        t1_science_residuals.append(float(row[0]))
        t1_cost_residuals.append(float(row[1]))

with open('report/data/residuals/set_2/t2_model_eval.csv', 'r+') as t2_in:
    reader = csv.reader(t2_in, delimiter=',')
    next(reader)
    for row in reader:
        t2_science_residuals.append(float(row[0]))
        t2_cost_residuals.append(float(row[1]))

with open('report/data/residuals/set_2/t3_model_eval.csv', 'r+') as t3_in:
    reader = csv.reader(t3_in, delimiter=',')
    next(reader)
    for row in reader:
        t3_science_residuals.append(float(row[0]))
        t3_cost_residuals.append(float(row[1]))

with open('report/data/residuals/set_2/limit_model_eval.csv', 'r+') as limit_in:
    reader = csv.reader(limit_in, delimiter=',')
    next(reader)
    for row in reader:
        limit_science_residuals.append(float(row[0]))
        limit_cost_residuals.append(float(row[1]))


sns.set(style="whitegrid")
ax = sns.boxplot(x="Sampling Method", y="Residual Science Benefit Error", data={'Sampling Method': [
                 'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Limit'], 'Residual Science Benefit Error': [lhc_science_residuals, ur_science_residuals, t1_science_residuals, t2_science_residuals, t3_science_residuals, limit_science_residuals]}, palette="Set2")
ax.set(xlabel='Sampling Method', ylabel='Residual Science Benefit Error')
plt.ylim(-0.3, .3)
plt.show()

ax = sns.boxplot(x="Sampling Method", y="Residual Cost Error", data={'Sampling Method': [
                 'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Limit'], 'Residual Cost Error': [lhc_cost_residuals, ur_cost_residuals, t1_cost_residuals, t2_cost_residuals, t3_cost_residuals, limit_cost_residuals]}, palette="Set2")
ax.set(xlabel='Sampling Method', ylabel='Residual Cost Error')
plt.ylim(-4000, 4000)
plt.show()
