import numpy as np
import csv
import sys
from keras.models import Sequential, load_model
from keras.layers import Dense
from keras.layers import Dropout
from keras import backend as K
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import train_test_split


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


def r_sq(y_true, y_pred):
    SS_res = np.sum(np.square(y_true-y_pred))
    SS_tot = np.sum(np.square(y_true - np.mean(y_true)))
    return K.cast_to_floatx(1 - SS_res/(SS_tot + 1e07))


def build_model(epochs):
    model = Sequential()
    model.add(Dense(1024, input_dim=60,
                    kernel_initializer='normal', activation='relu'))
    model.add(Dropout(0.2))
    model.add(Dense(512, kernel_initializer='normal', activation='relu'))
    model.add(Dropout(0.2))
    model.add(Dense(512, kernel_initializer='normal', activation='relu'))
    model.add(Dropout(0.2))
    model.add(Dense(1, kernel_initializer='normal'))
    model.compile(loss='logcosh', optimizer='adam')
    return model


def r_sq_score(regressor, x_test, y_test):
    preds = regressor.predict(x_test)
    score = r_sq(y_test, preds)
    return score


def norm_mse(y_true, y_pred):
    return K.mean(K.square(K.l2_normalize(y_pred) - K.l2_normalize(y_true)), axis=-1)


def build_from_fp(fp, savelocation):
    configs, cost, science = read_data(fp)

    configs = np.array(configs)
    cost = np.array(cost).reshape(-1, 1)

    xTr_s, xTe_s, yTr_s, yTe_s = train_test_split(configs, cost)

    cost_regressor = KerasRegressor(
        build_fn=build_model, epochs=20, batch_size=32, verbose=1)
    cost_regressor.fit(xTr_s, yTr_s)

    print("Cost r^2 score:", r_sq_score(cost_regressor, xTe_s, yTe_s))
    print("Cost mse score:", cost_regressor.score(xTe_s, yTe_s))

    cost_regressor.model.save(savelocation)


if __name__ == '__main__':
    fp = sys.argv[1]
    savelocation = sys.argv[2]
    build_from_fp(fp, savelocation)
