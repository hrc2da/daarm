import json


lhc_data = None
ur_data = None

with open('random_eval.json', 'r+') as infile:
    eval_data = json.load(infile)
    ur_data = {'data': [], 'label': 'ur_sample'}
    for config, obj in eval_data.items():
        ur_data['data'].append([float(obj[0]), float(obj[1])])

with open('ur_res.json', 'w+') as out:
    json.dump([ur_data, {"data": [], "label": "surrogate"}, {"data": [], "label": "model_iter"}], out)

with open('lhc_eval.json', 'r+') as infile:
    eval_data = json.load(infile)
    lhc_data = {'data': [], 'label': 'lhc_sample'}
    for config, obj in eval_data.items():
        lhc_data['data'].append([float(obj[0]), float(obj[1])])

with open('lhc_res.json', 'w+') as out:
    json.dump([lhc_data, {"data": [], "label": "surrogate"}, {"data": [], "label": "model_iter"}], out)

with open('combined.json', 'w+') as out:
    json.dump([lhc_data, ur_data, {"data": [], "label": "model_iter"}], out)
