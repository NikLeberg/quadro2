#!/usr/bin/python3
# -*- coding: utf-8 -*
import sys
import csv
import matplotlib.pyplot as plt

def main():
    with open(str(sys.argv[1])) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=";")
        rows = iter(readCSV)
        row = next(rows)
        # Header aus erster Zeile extrahieren
        headers = []
        for head in row:
            headers.append(head)
        # Datenspeicher erstellen
        data = []
        for _ in range(len(headers) - 1):
            data.append([[],[]])
        # über Zeilen iterieren
        while True:
            try:
                # Daten aus Zeile extrahieren
                # Time;pv/control/armed;parameter/control/throttle;pv/control/roll;pv/control/pitch;pv/control/xOut;pv/control/yOut
                row = next(rows)
                i = len(row) - 2
                data[i][0].append(float(row[0]) / 1000) # Zeit in s
                data[i][1].append(float(row[-1])) # Datenpunkt ist immer in letzter Zeile

            except (StopIteration) as e:
                print(e)
                break

        # Resultat
        plt.title("PID")
        for i in range(len(data)):
            skip = 0
            for exclude in ["rate", "Right", "Left"]:
                if exclude in headers[i + 1]:
                    skip = 1
            if skip:
                continue
            plt.plot(data[i][0], data[i][1], label=headers[i + 1])
        plt.ylim(-1.0, +1.0)
        plt.legend(loc = "upper right")
        plt.show()

main()
exit()
