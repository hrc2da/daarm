from keras.models import load_model
import numpy as np


class Translator:
    def __init__(self, path='models/mae_no_dropout_model.h5'):
        self.model = load_model(path)

    def translate(self, pt):
        return self.model.predict(np.array([pt.xpos, pt.ypos]).reshape(-1, 1))
