import csv
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from keras.layers import Dense, Dropout
from keras.models import Sequential, save_model, load_model
from keras.losses import mse, mean_absolute_error
from sklearn.model_selection import train_test_split


class CalibrationModel:
    def __init__(self):
        self.pts = []
        self.means = KMeans(n_clusters=69)
        self.model = self.build_model()
        self.load_data('data/blocks.csv')
        self.train()
        self.model.save('models/mse_model.h5')

    def load_data(self, fp):
        with open(fp, 'r+') as infile:
            for line in infile:
                x, y = line.split(',')
                self.pts.append([float(x), float(y)])
        self.organize_data()
        self.compute_targets()

    def organize_data(self):
        self.means.fit(np.array(self.pts))
        self.pruned_pts = self.means.cluster_centers_
        x_coords = self.pruned_pts[:, 0]
        y_coords = self.pruned_pts[:, 1]
        x_kmeans = KMeans(n_clusters=8)
        y_kmeans = KMeans(n_clusters=9)
        x_kmeans.fit(x_coords.reshape(-1, 1))
        y_kmeans.fit(y_coords.reshape(-1, 1))
        x_centers = x_kmeans.cluster_centers_
        y_centers = y_kmeans.cluster_centers_
        rows = [[] for _ in range(9)]
        for pt in self.pruned_pts:
            y_bin_distances = np.square(y_centers - pt[1])
            y_bin_i = np.argmin(y_bin_distances)
            rows[y_bin_i].append(pt)
        rows.sort(key=lambda e: e[0][1])
        training = []
        for row in rows:
            row.sort(key=lambda e: e[0])
            training += row
        self.training = np.array(training)

    def compute_targets(self):
        i_mult = 0.36
        j_mult = 0.24
        targets = []
        for i in range(9):
            for j in range(9):
                if i == 3:
                    continue
                else:
                    if j == 8 and (i == 2 or i == 3 or i == 4 or i == 5):
                        continue
                    else:
                        targets.append(np.array([(i+1)*i_mult, (j+1)*j_mult]))
        self.targets = np.array(targets)

    def graph_data(self):
        x = self.training[:, 0]
        y = self.training[:, 1]
        plt.scatter(x, y)
        plt.show()
        x = self.targets[:, 0]
        y = self.targets[:, 1]
        plt.scatter(x, y)
        plt.show()

    def build_model(self):
        model = Sequential()
        model.add(Dense(1024, input_dim=2, kernel_initializer='normal', activation='relu'))
        model.add(Dropout(0.2))
        model.add(Dense(512, kernel_initializer='normal', activation='relu'))
        model.add(Dense(512, kernel_initializer='normal', activation='relu'))
        model.add(Dense(2, kernel_initializer='normal'))
        model.compile(loss='mse', optimizer='adam')
        return model

    def train(self):
        input_tuio = self.training
        desired_output = self.targets

        input_tuio_train, input_tuio_test, desired_output_train, desired_output_test = train_test_split(
            input_tuio, desired_output, test_size=0.2)

        self.model.fit(input_tuio_train, desired_output_train, epochs=4000, verbose=1)
        preds = self.model.predict(input_tuio_test)
        print(preds, desired_output_test)
        print(self.model.evaluate(input_tuio_test, desired_output_test))


if __name__ == '__main__':
    c = CalibrationModel()
