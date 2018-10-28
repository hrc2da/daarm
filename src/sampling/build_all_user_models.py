from train_cost_model import build_from_fp as build_cost
from train_science_model import build_from_fp as build_science
import os

files = os.listdir('user_data/all_parsed')
for file in files:
    build_cost('user_data/all_parsed/' + file, 'report/models/user_models/'+file + '_cost')
    build_science('user_data/all_parsed/' + file, 'report/models/user_models/'+file + '_science')
