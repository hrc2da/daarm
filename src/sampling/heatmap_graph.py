import csv
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

data_files = ['report/data/lhc_train_test.csv', 'report/data/ur_train_test.csv', 'report/data/t1_train_test.csv',
              'report/data/t2_train_test.csv', 'report/data/t3_train_test.csv']

config_dist = []
for fp in data_files:
    curr_config_dist = np.zeros((5, 12))
    with open(fp, 'r+') as infile:
        reader = csv.reader(infile, delimiter=',')
        next(reader)
        for row in reader:
            config = np.array(list(map(float, list(row[0])))).reshape((5, 12))
            curr_config_dist = curr_config_dist + config

        curr_config_dist = curr_config_dist / np.max(curr_config_dist)
        config_dist.append(curr_config_dist)

for i in range(len(config_dist)):
    fig, ax = plt.subplots(figsize=[5, 6], facecolor='white')
    heatmap = ax.imshow(config_dist[i], cmap='hot_r', interpolation='nearest', vmin=0, vmax=1)
    ax.set_yticks(np.arange(0, 5))
    ax.set_xticks(np.arange(0, 12))
    ax.set_facecolor('white')
    ax.set_xticks(np.arange(-.5, 11.5, 1), minor=True)
    ax.set_yticks(np.arange(-.5, 5, 1), minor=True)
    # ax.set_title(treatments[i])
    ax.set_xticklabels(['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L'], fontsize=18)
    ax.set_yticklabels(['Orbit 1', 'Orbit 2', 'Orbit 3', 'Orbit 4', 'Orbit 5'], fontsize=18)
    # Gridlines based on minor ticks
    ax.grid(False)
    ax.grid(which='minor', color='w', linestyle='-', linewidth=2)
    ax.set_xlabel("Instrument")
    # ax.set_ylabel("Orbit")
    fig.colorbar(heatmap, orientation='horizontal')
    plt.title(data_files[i])
    plt.show()
