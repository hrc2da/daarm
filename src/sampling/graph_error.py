import matplotlib.pyplot as plt
import numpy as np
import csv
import seaborn as sns


def square_l(list):
    sq_l = []
    for elm in list:
        sq_l.append(elm * elm)
    return sq_l


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
expert_science_residuals = []
expert_cost_residuals = []

with open('report/data/residuals/lhc_model_eval.csv', 'r+') as lhc_in:
    reader = csv.reader(lhc_in, delimiter=',')
    next(reader)
    for row in reader:
        lhc_science_residuals.append(float(row[0]))
        lhc_cost_residuals.append(float(row[1]))

with open('report/data/residuals/ur_model_eval.csv', 'r+') as ur_in:
    reader = csv.reader(ur_in, delimiter=',')
    next(reader)
    for row in reader:
        ur_science_residuals.append(float(row[0]))
        ur_cost_residuals.append(float(row[1]))

with open('report/data/residuals/t1_model_eval.csv', 'r+') as t1_in:
    reader = csv.reader(t1_in, delimiter=',')
    next(reader)
    for row in reader:
        t1_science_residuals.append(float(row[0]))
        t1_cost_residuals.append(float(row[1]))

with open('report/data/residuals/t2_model_eval.csv', 'r+') as t2_in:
    reader = csv.reader(t2_in, delimiter=',')
    next(reader)
    for row in reader:
        t2_science_residuals.append(float(row[0]))
        t2_cost_residuals.append(float(row[1]))

with open('report/data/residuals/t3_model_eval.csv', 'r+') as t3_in:
    reader = csv.reader(t3_in, delimiter=',')
    next(reader)
    for row in reader:
        t3_science_residuals.append(float(row[0]))
        t3_cost_residuals.append(float(row[1]))

with open('report/data/residuals/expert_model_eval.csv', 'r+') as expert_in:
    reader = csv.reader(expert_in, delimiter=',')
    next(reader)
    for row in reader:
        expert_science_residuals.append(float(row[0]))
        expert_cost_residuals.append(float(row[1]))

science_residuals = [lhc_science_residuals, ur_science_residuals, t1_science_residuals,
                     t2_science_residuals, t3_science_residuals, expert_science_residuals]

cost_residuals = [lhc_cost_residuals, ur_cost_residuals, t1_cost_residuals,
                  t2_cost_residuals, t3_cost_residuals, expert_cost_residuals]

cost_err = list(map(square_l, cost_residuals))
science_err = list(map(square_l, science_residuals))


sns.set(style="whitegrid")
ax = sns.boxplot(x="Sampling Method", y="Squared Science Benefit Error", data={'Sampling Method': [
                 'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Expert Guided'], 'Squared Science Benefit Error': science_err}, palette="Set2")
ax.set(xlabel='Sampling Method', ylabel='Squared Science Benefit Error')
plt.ylim(0, .2)
plt.show()

ax = sns.boxplot(x="Sampling Method", y="Squared Cost Error", data={'Sampling Method': [
                 'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Expert Guided'], 'Squared Cost Error': cost_err}, palette="Set2")
ax.set(xlabel='Sampling Method', ylabel='Squared Cost Error')
plt.ylim(0, 40000)
plt.show()
