import csv
from matplotlib import pyplot as plt

predicted = []
evaluated = []
with open("results.csv", "rb") as ofile:
    csvreader = csv.reader(ofile, delimiter=',')
    for i, row in enumerate(csvreader):
        if i % 2 == 0:
            predicted.append((float(row[0]), float(row[1])))
        else:
            evaluated.append((float(row[0]), float(row[1])))

x, y = zip(*predicted)
plt.scatter(x, y, marker='o', label="Believed Outcomes")
x_, y_ = zip(*evaluated)
plt.scatter(x_, y_, marker='x', label="Actual Outcomes")
plt.legend(loc=0)
plt.show()
