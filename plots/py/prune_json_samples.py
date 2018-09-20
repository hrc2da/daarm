import json
import random


indices = [i for i in range(18480)]
random.shuffle(indices)
sampled_i = indices[0:300];

with open('../data/surrogate_learning.json', 'r+') as data_file:
    data = json.load(data_file)
    for model in data:
        model['data'] = [model['data'][i] for i in sampled_i];

with open('../data/surrograte_pruned.json', 'w+') as pruned_data:
    json.dump(data, pruned_data)
