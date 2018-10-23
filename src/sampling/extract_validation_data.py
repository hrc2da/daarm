import csv
from random import randrange

all_study_points = set()
files = ['report/data/lhc_train_test.csv', 'report/data/ur_train_test.csv',
         'report/data/t1_train_test.csv', 'report/data/t2_train_test.csv', 'report/data/t3_train_test.csv']
for file in files:
    with open(file, 'r+') as infile:
        reader = csv.reader(infile, delimiter=',')
        next(reader)
        for row in reader:
            all_study_points.add(row[0])

candidate_eoss_pts = []
with open('user_data/all_samples_eoss.csv', 'r+') as infile:
    reader = csv.reader(infile, delimiter=',')
    next(reader)
    for row in reader:
        candidate_eoss_pts.append(row)

samples = []
while len(samples) < 100:
    random_index = randrange(0, len(candidate_eoss_pts))
    if candidate_eoss_pts[random_index][0] not in all_study_points and float(candidate_eoss_pts[random_index][1]) != -1 and float(candidate_eoss_pts[random_index][2]) != -1:
        samples.append(candidate_eoss_pts[random_index])

with open('report/data/model_validation_data.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    writer.writerow(['config', 'science', 'cost'])
    for row in samples:
        writer.writerow(row)
