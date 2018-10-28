import matplotlib.pyplot as plt
import numpy as np
import csv
import seaborn as sns
import pandas as pd

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

with open('report/data/residuals/all_user/t1_cost_residuals.csv', 'r+') as infile:
    reader = csv.reader(infile, delimiter=',')
    next(reader)
    for row in reader:
        t1_cost_residuals.append(float(row[0]))

with open('report/data/residuals/all_user/t1_science_residuals.csv', 'r+') as infile:
    reader = csv.reader(infile, delimiter=',')
    next(reader)
    for row in reader:
        t1_science_residuals.append(float(row[0]))

with open('report/data/residuals/all_user/t2_cost_residuals.csv', 'r+') as infile:
    reader = csv.reader(infile, delimiter=',')
    next(reader)
    for row in reader:
        t2_cost_residuals.append(float(row[0]))

with open('report/data/residuals/all_user/t2_science_residuals.csv', 'r+') as infile:
    reader = csv.reader(infile, delimiter=',')
    next(reader)
    for row in reader:
        t2_science_residuals.append(float(row[0]))

with open('report/data/residuals/all_user/t3_cost_residuals.csv', 'r+') as infile:
    reader = csv.reader(infile, delimiter=',')
    next(reader)
    for row in reader:
        t3_cost_residuals.append(float(row[0]))

with open('report/data/residuals/all_user/t3_science_residuals.csv', 'r+') as infile:
    reader = csv.reader(infile, delimiter=',')
    next(reader)
    for row in reader:
        t3_science_residuals.append(float(row[0]))

with open('report/data/residuals/set_2/limit_model_eval.csv', 'r+') as infile:
    reader = csv.reader(infile, delimiter=',')
    next(reader)
    for row in reader:
        limit_science_residuals.append(float(row[0]))
        limit_cost_residuals.append(float(row[1]))

# sns.set(style="whitegrid")
# ax = sns.boxplot(x="Sampling Method", y="Residual Science Benefit Error", data={'Sampling Method': [
#                  'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Limit'], 'Residual Science Benefit Error': [lhc_science_residuals, ur_science_residuals, t1_science_residuals, t2_science_residuals, t3_science_residuals, limit_science_residuals]}, palette="Set2")
# ax = sns.swarmplot(x="Sampling Method", y="Residual Science Benefit Error", data={'Sampling Method': [
#     'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Limit'], 'Residual Science Benefit Error': [lhc_science_residuals, ur_science_residuals, t1_science_residuals, t2_science_residuals, t3_science_residuals, limit_science_residuals]}, palette="Set2")

# ax.set(xlabel='Sampling Method', ylabel='Residual Science Benefit Error')
# plt.ylim(-0.3, .3)
# plt.show()

# ax = sns.boxplot(x="Sampling Method", y="Residual Cost Error", data={'Sampling Method': [
#                  'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Limit'], 'Residual Cost Error': [lhc_cost_residuals, ur_cost_residuals, t1_cost_residuals, t2_cost_residuals, t3_cost_residuals, limit_cost_residuals]}, palette="Set2")
# ax = sns.swarmplot(x="Sampling Method", y="Residual Cost Error", data={'Sampling Method': [
#     'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Limit'], 'Residual Cost Error': [lhc_cost_residuals, ur_cost_residuals, t1_cost_residuals, t2_cost_residuals, t3_cost_residuals, limit_cost_residuals]}, palette="Set2")

# ax.set(xlabel='Sampling Method', ylabel='Residual Cost Error')
# plt.ylim(-4000, 4000)
# plt.show()

sns.set(style="whitegrid")
# ax = sns.boxplot(x="Sampling Method", y="Residual Science Benefit Error", data={'Sampling Method': [
#                  'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Limit'], 'Residual Science Benefit Error': [lhc_science_residuals, ur_science_residuals, t1_science_residuals, t2_science_residuals, t3_science_residuals, limit_science_residuals]}, palette="Set2", orient='v')
# ax.set(xlabel='Sampling Method', ylabel='Residual Science Benefit Error')
science_df1 = pd.DataFrame({"Latin Hypercube": lhc_science_residuals,
                            "Uniform Random": ur_science_residuals,
                            "Reference": limit_science_residuals})
science_df2 = pd.DataFrame({
                            "User Guided (T1)": t1_science_residuals,
                            "Agent Guided (T2)": t2_science_residuals,
                            "Collaboratively Guided (T3)": t3_science_residuals
})
science_df = pd.concat([science_df1,science_df2], ignore_index=False, axis=1)
ax = sns.boxplot(data=science_df, order = ['Latin Hypercube', 'Uniform Random', 'User Guided (T1)', 'Agent Guided (T2)', 'Collaboratively Guided (T3)', 'Reference'])                            
#ax = sns.swarmplot(data=science_df, color='0.25', order = ['Latin Hypercube', 'Uniform Random', 'User Guided (T1)', 'Agent Guided (T2)', 'Collaboratively Guided (T3)', 'Reference'])
ax.set(xlabel='Sampling Method', ylabel='Residual Science Benefit Error')
plt.ylim(-0.3, .3)
plt.show()

# ax = sns.boxplot(x="Sampling Method", y="Residual Cost Error", data={'Sampling Method': [
#                  'Latin Hypercube', 'Uniform Random', 'User Guided t1', 'Agent Guided t2', 'Collaboratively Explored t3', 'Limit'], 'Residual Cost Error': [lhc_cost_residuals, ur_cost_residuals, t1_cost_residuals, t2_cost_residuals, t3_cost_residuals, limit_cost_residuals]}, palette="Set2", orient = 'v')
# ax.set(xlabel='Sampling Method', ylabel='Residual Cost Error')
# plt.ylim(-4000, 4000)
# plt.show()
cost_df = pd.DataFrame({"Latin Hypercube": lhc_cost_residuals,
                            "Uniform Random": ur_cost_residuals,
                            "User Guided (T1)": t1_cost_residuals,
                            "Agent Guided (T2)": t2_cost_residuals,
                            "Collaboratively Guided (T3)": t3_cost_residuals,
                            "Reference": limit_cost_residuals})
ax = sns.violinplot(data=cost_df, order = ['Latin Hypercube', 'Uniform Random', 'User Guided (T1)', 'Agent Guided (T2)', 'Collaboratively Guided (T3)', 'Reference'])
ax.set(xlabel='Sampling Method', ylabel='Residual Cost Benefit Error')
plt.ylim(-6000, 6000)
plt.show()
