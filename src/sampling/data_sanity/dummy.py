import csv
import json

with open('random_results.txt', 'r+') as infile:
    cvsr = csv.reader(infile, delimiter=',')
    results = []
    data = {'data': [], 'label': 'test'}
    for row in cvsr:
        data['data'].append([float(row[0]), float(row[1])])
    results.append(data)

with open('ur_res.json', 'w+') as out:
    json.dump(data, out)
