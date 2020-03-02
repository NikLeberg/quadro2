#!/usr/bin/python3
# -*- coding: utf-8 -*
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import math
import warnings
warnings.simplefilter('error', RuntimeWarning)

# Kalman Filter
# Physik basierend auf konstanter Geschwindigkeit
# bei Beschleunigungsmessung wird Status vorausgesagt, resp. Geschwindigkeit integriert per Input
# bei den anderen Messungen wird nur korrigiert.

def main():
    dT = 0
    x = np.matrix([[0.0], [0.0]]) # Höhe, Geschwindigkeit
    z = np.matrix([[0.0], [0.0]]) # GPS Position, GPS Geschwindigkeit
    P = np.diag([0.0, 0.1]) # anfängliche Unsicherheit 0.1
    H = np.matrix([[0.0, 0.0],
                   [0.0, 0.0]])
    rP = 5.0 / 2
    rS = 0.05 / 2
    F = np.identity(2) # Physikmodell (muss um dT in (0,1) angepasst werden)
    G = np.matrix([[0.0], [0.0]]) # Inputmodell (muss um dT angepasst werden)
    I = np.identity(2)
    t = [0.0]
    f = [[0.0], [0.0], [0.0]]
    v = [[0.0], [0.0], [0.0]]
    p = [[0.0], [0.0], [0.0]]
    s = [[0.0], [0.0], [0.0]]
    a = [[0.0], [0.0], [0.0]]
    num = 0
    numSkip = 0

    with open(str(sys.argv[1])) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=';')
        rows = iter(readCSV)
        row = next(rows)
        # Offsets aus erster Zeile extrahieren
        lastTimestamp = int(row[1])
        # über Zeilen iterieren
        while True:
            try:
                # Daten aus Zeile extrahieren
                row = next(rows)
                num = num + 1
                mType = row[2] # art der Messung: A, P oder S
                timestamp = int(row[1])
                dT = (timestamp - lastTimestamp) / 1000 / 1000 # vergangene Zeit (in s)
                if dT < 0:
                    numSkip = numSkip + 1
                    print("skip", mType)
                    continue
                lastTimestamp = timestamp
                if mType == 'A': # Beschleunigung x = 3, y = 4
                    mValue = float(row[4])
                elif mType == 'P': # Position
                    mValue = float(row[4])
                    mAccuracy = float(row[6])
                elif mType == 'S': # Geschwindigkeit
                    mValue = float(row[4])
                    mAccuracy = float(row[5])
                else:
                    print("Messung übersprungen:", mType)
                    continue


                # Predict
                if mType == 'A': # Beschleunigung -> Voraussage
                    # Matrizen anpassen
                    F.itemset((0, 1), dT) # Physik
                    G.itemset((0, 0), 0.5 * dT**2) # Input
                    G.itemset((1, 0), dT) # Input
                    # Beschleunigung zur Geschwindigkeit integrieren
                    xHat = F * x + G * mValue
                    Q = G * G.T * (abs(mValue) + 0.35)
                    PHat = (F * P * F.T) + Q
                    x = xHat
                    P = PHat
                    #continue # nur Vorraussage bei Beschleunigung
                else:
                    if mType == 'P': # Position -> Korrektur
                        R = np.diag([mAccuracy**2, 0.0])
                        z.itemset(0, mValue)
                        H = np.matrix([[1.0, 0.0],
                                       [0.0, 0.0]])
                    elif mType == 'S': # Geschwindigkeit -> Korrektur
                        R = np.diag([0.0, mAccuracy**2])
                        z.itemset(1, mValue)
                        H = np.matrix([[0.0, 0.0],
                                       [0.0, 1.0]])

                    # Gain rechnen und Korrigieren
                    S = (H * P * H.T) + R
                    if S.item((0, 0)) != 0:
                        S.itemset((0, 0), 1.0 / S.item((0, 0)))
                    if S.item((1, 1)) != 0:
                        S.itemset((1, 1), 1.0 / S.item((1, 1)))
                    K = (P * H.T) * S #np.linalg.pinv(S)
                    P = (I - (K * H)) * P
                    x = x + K * (z - (H * x))

                # speichere für Plot
                t.append(t[-1] + dT)
                f[0].append(x.item(0))
                f[1].append(f[0][-1] - 2 * np.sqrt(abs(P[0, 0])))
                f[2].append(f[0][-1] + 2 * np.sqrt(abs(P[0, 0])))
                v[0].append(x.item(1))
                v[1].append(v[0][-1] - 2 * np.sqrt(abs(P[1, 1])))
                v[2].append(v[0][-1] + 2 * np.sqrt(abs(P[1, 1])))
                if mType == 'A':
                    a[0].append(mValue)
                else:
                    a[0].append(a[0][-1])
                p[0].append(z.item(0))
                s[0].append(z.item(1))
                

            except StopIteration:
                break
            except np.linalg.LinAlgError:
                print("Singuläre Matrix in Schritt", num)
                break


        print(numSkip, "von", num, "Messungen übersprungen", 100.0 / num * numSkip)
        # Resultat
        plt.title('Fusion')
        if len(sys.argv) > 2:
            toPlot = sys.argv[2]
        else:
            toPlot = "fvpsa2"
        if "f" in toPlot:
            plt.plot(t, f[0], 'b', label='Fusion')
            if "fe" in toPlot:
                plt.plot(t, f[1], 'b--')
                plt.plot(t, f[2], 'b--')
        if "v" in toPlot:
            plt.plot(t, v[0], 'r', label='Geschwindigkeit')
            if "ve" in toPlot:
                plt.plot(t, v[1], 'r--')
                plt.plot(t, v[2], 'r--')
        if "p" in toPlot:
            plt.plot(t, p[0], 'g', label='Position')
        if "s" in toPlot:
            plt.plot(t, s[0], 'y', label='Speed')
        if "a" in toPlot:
            plt.plot(t, a[0], 'm', label='Beschleunigung')
        if "2" in toPlot:
            plt.ylim(-2.0, +2.0)
        plt.legend()
        plt.show()

        #plt.matshow(P, cmap='binary')
        #plt.title('Warscheinlichkeitsverteilung')
        #plt.show()
        #plt.matshow(K, cmap='binary')
        #plt.title('Kalman Gain')
        #plt.show()

main()
exit()
