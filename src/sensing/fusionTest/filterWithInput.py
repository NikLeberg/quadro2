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
# per Input wird die Beschleunigung zur Geschwindigkeit integriert

def main():
    dT = 0
    x = np.matrix([[0.0], [0.0]]) # Höhe, Geschwindigkeit
    z = np.matrix([[0.0], [0.0]]) # Ultraschall, Barometer
    P = np.diag([0.1, 0.1]) # anfängliche Unsicherheit 0.1
    H = np.matrix([[1.0, 0.0],
                   [1.0, 0.0]])
    rU = 0.005
    rB = 1.0
    R = np.diag([rU**2, rB**2]) # Unsicherheit der Messung (per Messung)
    F = np.identity(2) # Physikmodell (muss um dT in (0,1) angepasst werden)
    G = np.matrix([[0.0], [0.0]]) # Inputmodell (muss um dT angepasst werden)
    I = np.identity(2)
    t = [0.0]
    f = [0.0]
    fl = [0.0]
    fu = [0.0]
    u = [0.0]
    b = [0.0]
    v = [0.0]
    vl = [0.0]
    vu = [0.0]
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
                lastTimestamp = timestamp
                if mType == 'A': # Beschleunigung
                    mValue = float(row[5])
                elif mType == 'U': # Ultraschall
                    mValue = float(row[3]) - offsetUltrasonic
                elif mType == 'B': # Barometer
                    mValue = float(row[3]) - offsetBarometer

                # Matrizen anpassen
                F.itemset((0, 1), dT) # Physik
                G.itemset((0, 0), 0.5 * dT**2) # Input
                G.itemset((1, 0), dT) # Input

                # Predict
                if mType == 'A':
                    #if (abs(mValue) < 0.35): continue # kleine Beschleunigungen ignorieren
                    xHat = F * x + G * mValue # Beschleunigung zur Geschwindigkeit integrieren
                    Q = G * G.T * mValue
                    PHat = (F * P * F.T) + Q
                    x = xHat
                    P = PHat
                    continue # nur Vorraussage bei Beschleunigung
                # Update
                else:
                    xHat = F * x
                    PHat = F * P * F.T

                # zurückgelegte Distanz (gemäss Voraussage)
                dX = abs(xHat.item(0) - x.item(0))

                # Messunsicherheit um dX erhöhen
                R[(0, 0)] = math.sqrt(R[(0, 0)] + dX)**2
                R[(1, 1)] = math.sqrt(R[(1, 1)] + dX)**2

                # Messung verarbeiten
                if mType == 'U':
                    z.itemset(0, mValue)
                    R[(0, 0)] = rU**2
                elif mType == 'B':
                    z.itemset(1, mValue)
                    R[(1, 1)] = rB**2

                # Gain rechnen und Korrigieren
                S = (H * P * H.T) + R
                K = (P * H.T) * np.linalg.inv(S)
                P = (I - (K * H)) * PHat
                x = xHat + K * (z - (H * xHat))

                # speichere für Plot
                t.append(t[-1] + dT)
                f.append(x.item(0))
                fl.append(f[-1] - 2 * np.sqrt(abs(P[0, 0])))
                fu.append(f[-1] + 2 * np.sqrt(abs(P[0, 0])))
                v.append(x.item(1))
                print(v[-1], P[1, 1])
                vl.append(v[-1] - 2 * np.sqrt(abs(P[1, 1])))
                vu.append(v[-1] + 2 * np.sqrt(abs(P[1, 1])))
                u.append(z.item(0))
                b.append(z.item(1))

            except (StopIteration) as e: # np.linalg.LinAlgError
                break


        # Resultat
        plt.title('Fusion')
        plt.plot(t, f, 'b', label='Fusion')
        plt.plot(t, fl, 'b--')
        plt.plot(t, fu, 'b--')
        plt.plot(t, v, 'r', label='Geschwindigkeit')
        plt.plot(t, vl, 'r--')
        plt.plot(t, vu, 'r--')
        plt.plot(t, u, label='Ultraschall')
        plt.plot(t, b, label='Barometer')
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
