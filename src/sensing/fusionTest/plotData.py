#!/usr/bin/python3
# -*- coding: utf-8 -*
import sys
import csv
import matplotlib.pyplot as plt

def main():
    t = [0.0]
    a = [0.0]
    u = [0.0]
    b = [0.0]
    v = [0.0]
    lastA = 0
    offsetUltrasonic = 0.0
    offsetBarometer = 0.0

    with open(str(sys.argv[1])) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=';')
        rows = iter(readCSV)
        row = next(rows)
        # Offsets aus erster Zeile extrahieren
        lastTimestamp = int(row[1])
        offsetUltrasonic = float(row[3])
        offsetBarometer = float(row[5])
        # über Zeilen iterieren
        while True:
            try:
                # Daten aus Zeile extrahieren
                row = next(rows)
                mType = row[2] # art der Messung: A, U oder B
                timestamp = int(row[1])
                dT = (timestamp - lastTimestamp) / 1000 / 1000 # vergangene Zeit (in s)
                if dT <= 0:
                    print("skip", mType)
                    #continue
                    dT = 0.000001
                lastTimestamp = timestamp
                if mType == 'A': # Beschleunigung
                    mValue = float(row[5])
                elif mType == 'U': # Ultraschall
                    mValue = float(row[3]) - offsetUltrasonic
                elif mType == 'B': # Barometer
                    mValue = float(row[3]) - offsetBarometer

                # Messung verarbeiten
                if mType == 'U':
                    u.append(mValue)
                else:
                    u.append(u[-1])
                if mType == 'B':
                    b.append(mValue)
                else:
                    b.append(b[-1])
                if mType == 'A':
                    dTA = (timestamp - lastA) / 1000 / 1000
                    lastA = timestamp
                    a.append(mValue)
                    v.append(v[-1] + (mValue * dTA))
                    print(dT - dTA)
                else:
                    a.append(a[-1])
                    v.append(v[-1])

                # speichere für Plot
                t.append(t[-1] + dT)

            except (StopIteration) as e: # np.linalg.LinAlgError
                print(e)
                break

        # Resultat
        plt.title('Fusion')
        if len(sys.argv) > 2:
            toPlot = sys.argv[2]
        else:
            toPlot = "avub"
        if "a" in toPlot:
            plt.plot(t, a, 'b', label='Beschleunigung')
        if "v" in toPlot:
            plt.plot(t, v, 'r', label='Geschwindigkeit')
        if "u" in toPlot:
            plt.plot(t, u, 'g', label='Ultraschall')
        if "b" in toPlot:
            plt.plot(t, b, 'y', label='Barometer')
        plt.ylim(-2.0, +2.0)
        plt.legend()
        plt.show()

main()
exit()
