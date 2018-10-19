import matplotlib.pyplot as plt
import numpy as np
import csv
import seaborn as sns

lhc_science_residuals = []
lhc_cost_residuals = []
ur_science_residuals = []
ur_cost_residuals = []
ga_science_residuals = []
ga_cost_residuals = []

with open('data/lhc_model_eval.csv', 'r+') as lhc_in:
    reader = csv.reader(lhc_in, delimiter=',')
    next(reader)
    for row in reader:
        lhc_science_residuals.append(float(row[0]))
        lhc_cost_residuals.append(float(row[1]))

with open('data/ur_model_eval.csv', 'r+') as ur_in:
    reader = csv.reader(ur_in, delimiter=',')
    next(reader)
    for row in reader:
        ur_science_residuals.append(float(row[0]))
        ur_cost_residuals.append(float(row[1]))

with open('data/ga_model_eval.csv', 'r+') as ga_in:
    reader = csv.reader(ga_in, delimiter=',')
    next(reader)
    for row in reader:
        ga_science_residuals.append(float(row[0]))
        ga_cost_residuals.append(float(row[1]))


sns.set(style="whitegrid")
ax = sns.boxplot(x="Sampling Method", y="Residual Science Benefit Error", data={'Sampling Method': [
                 'Latin Hypercube', 'Uniform Random', 'User Guided'], 'Residual Science Benefit Error': [lhc_science_residuals, ur_science_residuals, ga_science_residuals]}, palette="Set2")
ax.set(xlabel='Sampling Method', ylabel='Residual Science Benefit Error')
plt.ylim(-0.2, .2)
plt.show()

ax = sns.boxplot(x="Sampling Method", y="Residual Cost Error", data={'Sampling Method': [
                 'Latin Hypercube', 'Uniform Random', 'User Guided'], 'Residual Cost Error': [lhc_cost_residuals, ur_cost_residuals, ga_cost_residuals]}, palette="Set2")
ax.set(xlabel='Sampling Method', ylabel='Residual Cost Error')
plt.ylim(-3000, 2000)
plt.show()
