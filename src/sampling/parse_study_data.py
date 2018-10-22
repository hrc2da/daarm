import csv

usr_data = []
with open('user_data/treatment_2.csv', 'r+') as infile:
    reader = csv.reader(infile, delimiter=',')
    next(reader)
    line_num = 0
    for row in reader:
        if len(usr_data) == 95:
            break
        if row[2].strip() == 'AGENT_EVENT':
            usr_data.append([row[3], row[4], row[5]])

with open('report/data/t2_train_test.csv', 'w+') as outfile:
    writer = csv.writer(outfile, delimiter=',')
    for row in usr_data:
        writer.writerow(row)
