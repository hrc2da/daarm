import json
import csv
import numpy as np

lhc_objectives = []
ur_objectives = []

lhc_configs = []
ur_configs = []


def listify(string_configs):
    l_configs = []
    for config in string_configs:
        config = config[1:-1].split(',')
        print(config)
        l_config = ''
        for elm in config:
            if elm.strip() == 'true':
                l_config += '1'
            else:
                l_config += '0'
        l_configs.append(l_config)

    return l_configs


with open("latin_hyper_cube_samples_init.json", "r+") as lhc_in:
    dat = json.load(lhc_in)
    lhc_configs = listify(dat["0"])

with open("uniform_random_samples_init.json", "r+") as ur_in:
    dat = json.load(ur_in)
    ur_configs = listify(dat["0"])

with open('lhc_res.json', 'r+') as lhc_res:
    dat = json.load(lhc_res)
    lhc_objectives = dat[0]['data']

with open('ur_res.json', 'r+') as ur_res:
    dat = json.load(ur_res)
    ur_objectives = dat[0]['data']

lhc_dat = []
ur_dat = []


for i in range(len(lhc_configs)):
    lhc_dat.append([lhc_configs[i], lhc_objectives[i][0], lhc_objectives[i][1]])

for i in range(len(ur_configs)):
    ur_dat.append([ur_configs[i], ur_objectives[i][0], ur_objectives[i][1]])

with open('lhc_train_test.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['config', 'science_benefit', 'cost'])
    for row in lhc_dat:
        writer.writerow(row)

with open('ur_train_test.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['config', 'science_benefit', 'cost'])
    for row in ur_dat:
        writer.writerow(row)
