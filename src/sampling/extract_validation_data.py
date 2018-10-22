import csv

all_study_points = set()
files = ['report/data/lhc_train_test.csv', 'report/data/ur_train_test.csv', 'report/data/t1_train_test.csv', 'report/data/t2_train_test.csv', 'report/data/t3_train_test.csv']
for file in files:
    with open(file, 'r+') as infile:
        reader = csv.reader(infile, delimiter= ',')
        next(reader)
        for row in reader:
            all_study_points.add(row[0])

print(len(all_study_points))
