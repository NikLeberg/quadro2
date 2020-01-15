#!/usr/bin/python3
# -*- coding: utf-8 -*
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import math

def inv(m):
    a, b = m.shape
    if a != b:
        raise ValueError("Only square matrices are invertible.")

    i = np.eye(a, a)
    return np.linalg.lstsq(m, i, rcond=-1)[0]

def main():
    dT = 0
    x = np.matrix([[0.0],
                  [0.0],
                  [0.0]]) # Höhe, Geschwindigkeit, Beschleunigung
    z = np.matrix([[0.0],
                  [0.0],
                  [0.0]]) # Beschleunigung, Ultraschall, Barometer
    P = np.matrix([[0.1, 0.0, 0.0],
                  [0.0, 0.1, 0.0],
                  [0.0, 0.0, 0.1]]) # anfängliche Unsicherheit 0.1
    Q = np.matrix([[10.0**2, 0.0, 0.0],
                  [0.0, 10.0**2, 0.0],
                  [0.0, 0.0, 10.0**2]]) # Unsicherheit der Vorraussage
    H = np.matrix([[0.0, 0.0, 1.0],
                  [1.0, 0.0, 0.0],
                  [1.0, 0.0, 0.0]])
    R = np.matrix([[0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0]])
    I = np.identity(3)
    tLast = 0
    t = [0.0]
    f = [0.0]
    v = [0.0]
    a = [0.0]
    u = [0.0]
    b = [0.0]
    s = 0.0
    num = 0
    offsetUltrasonic = 0.0
    offsetBarometer = 0.0

    with open(str(sys.argv[1])) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=';')
        for row in readCSV:
            if tLast == 0:
                # erste Zeile
                tLast = int(row[1])
                # Offsets
                offsetUltrasonic = float(row[3])
                offsetBarometer =float(row[5])
            else:
                num = num + 1
                #if num > 1000:
                    #break
                dT = (int(row[1]) - tLast) / 1000 / 1000
                s = s + dT
                tLast = int(row[1])
                F = np.matrix([[1, dT, 0.5 * dT * dT], [0, 1, dT], [0, 0, 1]]) # Physikmodell anpassen
                xHat = F * x # Predict
                PHat = F * P * F.T + Q
                # Messungsunwarscheinlichkeit erhöhen
                dM = abs(xHat.item(0) - x.item(0))
                R[(0, 0)] = math.sqrt(R[(0, 0)] + (dM / (0.5 * dT * dT)))**2
                R[(1, 1)] = math.sqrt(R[(1, 1)] + dM)**2
                R[(2, 2)] = math.sqrt(R[(2, 2)] + dM)**2
                # Zurücksetzen basierend auf Messung
                if row[2] == 'A':
                    R[(0, 0)] = 0.35**2 # 0.35
                    z.itemset(0, float(row[5]))
                elif row[2] == 'U':
                    R[(1, 1)] = 0.005**2 # 0.005
                    z.itemset(1, float(row[3]) - offsetUltrasonic)
                elif row[2] == 'B':
                    R[(2, 2)] = 1.0**2
                    z.itemset(2, float(row[3]) - offsetBarometer)
                # Gain rechnen
                S = H * P * H.T + R
                K = (P * H.T) * inv(S)
                P = (I - K * H) * PHat
                x = xHat + K * (z - H * xHat)
                # speichere für Plot
                t.append(s)
                f.append(x.item(0))
                v.append(x.item(1))
                a.append(z.item(0))
                u.append(z.item(1))
                b.append(z.item(2))

        plt.title('Fusion')
        plt.plot(t, f, label='Fusion')
        #plt.plot(t, v, label='Geschwindigkeit')
        plt.plot(t, a, label='Beschleunigung')
        plt.plot(t, u, label='Ultraschall')
        plt.plot(t, b, label='Barometer')
        #plt.ylim(-1.0, +1.0)
        plt.legend()
        plt.show()
        #plt.matshow(P, cmap='binary')
        #plt.title('Warscheinlichkeitsverteilung')
        #plt.show()
        plt.matshow(K, cmap='binary')
        plt.title('Kalman Gain')
        plt.show()

        # ToDo
        # Beschleunigung rechnung prüfen
        # Kalman Formeln prüfen
        # inverse prüfen

main()