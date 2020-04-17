#!/usr/bin/python3
# -*- coding: utf-8 -*
import sys
import csv
import matplotlib.pyplot as plt

def main():
    time = [0.0]
    ist = [0.0]
    soll = [0.0]
    out = [0.0]

    with open(str(sys.argv[1])) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=';')
        rows = iter(readCSV)
        row = next(rows)
        # Header aus erster Zeile extrahieren
        headIst = row[1]
        headSoll = row[2]
        headOut = row[3]
        row = next(rows)
        lastTimestamp = float(row[0])
        # über Zeilen iterieren
        while True:
            try:
                # Daten aus Zeile extrahieren
                row = next(rows)
                timestamp = float(row[0])
                dT = (timestamp - lastTimestamp) / 1000 # vergangene Zeit (in s)
                lastTimestamp = timestamp
                if (len(row) > 1 and row[1] != ''):
                    ist.append(float(row[1]))
                    soll.append(soll[-1])
                    out.append(out[-1])
                if (len(row) > 2 and row[2] != ''):
                    ist.append(ist[-1])
                    soll.append(float(row[2]))
                    out.append(out[-1])
                if (len(row) > 3 and row[3] != ''):
                    ist.append(ist[-1])
                    soll.append(soll[-1])
                    out.append(float(row[3]))
                # speichere für Plot
                time.append(time[-1] + dT)

            except (StopIteration) as e: # np.linalg.LinAlgError
                print(e)
                break

        # Resultat
        plt.title('PID')
        plt.plot(time, ist, label=headIst)
        plt.plot(time, soll, label=headSoll)
        plt.plot(time, out, label=headOut)
        plt.legend()
        plt.show()

        # y-2D = 0.63158
        # y-pu = 0.65895
        # y-A  = 0.322361
        # Ku = 4 * D / (A*pi) = 1.2472858560183056407556944825312
        # P: 0.6 * Ku
        # I: 1.2 * Ku / Pu
        # D: 0.075 * Ku * Pu
        # PID = 0.75 / 2.27 / 0.062

main()
exit()
