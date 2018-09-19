import json
import csv
import random

predicted = []
evaluated = []
with open("../data/results.csv", "r+") as results:
    csvreader = csv.reader(results, delimiter=',')
    for (i, row) in enumerate(csvreader):
        if i % 2 == 0:
            predicted.append((float(row[0]), float(row[1])))
        else:
            evaluated.append((float(row[0]), float(row[1])))

fmt_data = [{'label': 'EOSS', 'data': []}, {'label': 'Pretrained', 'data': []}]
for pt in evaluated:
    fmt_data[0]['data'].append(pt)

for pt in predicted:
    fmt_data[1]['data'].append(pt)

for i in range(22):
    model = {'label': 'model'+str(i), 'data': []}
    for _ in range(92):
        model['data'].append([random.random(), random.random() * 20000])
    fmt_data.append(model)

with open("../data/formatted_results.json", "w+") as json_out:
    json.dump(fmt_data, json_out)
