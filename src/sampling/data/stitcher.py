import json
import csv
import numpy as np

lhc_dat = []
ur_dat = []


def listify(string_config):
    l_config = ''
    for elm in string_config[1:-1].split(','):
        if elm.strip() == 'true':
            l_config += '1'
        else:
            l_config += '0'

    return l_config


with open("lhc_eval.json", "r+") as lhc_in:
    dat = json.load(lhc_in)
    for config, obj in dat.items():
        lhc_dat.append([listify(config), obj[0], obj[1]])

with open("random_eval.json", "r+") as ur_in:
    dat = json.load(ur_in)
    for config, obj in dat.items():
        ur_dat.append([listify(config), obj[0], obj[1]])

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
