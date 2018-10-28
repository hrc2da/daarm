import csv
import os

fp_l = os.listdir('user_data/all_logs')
user_num = 0

for fp in fp_l:
    user_num += 1
    with open('user_data/all_logs/'+fp, 'r+') as infile:
        reader = csv.reader(infile, delimiter=',')
        t1 = []
        t2 = []
        t3 = []
        curr_treatment = None
        for row in reader:
            try:
                event_type = row[2].strip()
                if event_type == 'CLEAR_SCREEN':
                    curr_treatment = None
                elif event_type == 'TREATMENT_1':
                    curr_treatment = 1
                elif event_type == 'TREATMENT_2':
                    curr_treatment = 2
                elif event_type == 'TREATMENT_3':
                    curr_treatment = 3

                if curr_treatment == 1 and len(t1) < 95:
                    if event_type == 'EXPLORE_EVENT':
                        t1.append([row[3], row[4], row[5]])
                elif curr_treatment == 2 and len(t2) < 95:
                    if event_type == 'AGENT_EVENT':
                        t2.append([row[3], row[4], row[5]])
                elif curr_treatment == 3 and len(t3) < 95:
                    if event_type == 'AGENT_EVENT' or event_type == 'EXPLORE_EVENT':
                        t3.append([row[3], row[4], row[5]])
            except:
                print(row)

        all_data = [('t1', t1), ('t2', t2), ('t3', t3)]
        for treatment, treatment_data in all_data:
            with open('user_data/all_parsed/user_'+str(user_num)+'_'+treatment, 'w+') as outfile:
                writer = csv.writer(outfile, delimiter=',')
                writer.writerow(['config', 'science', 'cost'])
                for row in treatment_data:
                    writer.writerow(row)


# with open('user_data/treatment_2.csv', 'r+') as infile:
#     reader = csv.reader(infile, delimiter=',')
#     next(reader)
#     line_num = 0
#     for row in reader:
#         if len(usr_data) == 95:
#             break
#         if row[2].strip() == 'AGENT_EVENT':
#             usr_data.append([row[3], row[4], row[5]])
