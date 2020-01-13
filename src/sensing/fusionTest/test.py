import csv
import numpy as np
import matplotlib.pyplot as plt

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
    Q = np.matrix([[0.1, 0.0, 0.0],
                  [0.0, 0.1, 0.0],
                  [0.0, 0.0, 0.1]]) # Unsicherheit der Vorraussage
    H = np.matrix([[0.0, 0.0, 1.0],
                  [1.0, 0.0, 0.0],
                  [1.0, 0.0, 0.0]])
    R = np.matrix([[0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0]])
    I = np.identity(3)
    tLast = 0
    m = [0.0]
    u = [0.0]
    b = [0.0]
    a = [0.0]
    num = 0

    with open('data-test2.csv') as csvfile:
        readCSV = csv.reader(csvfile, delimiter=';')
        for row in readCSV:
            if tLast == 0:
                tLast = int(row[1])
            else:
                num = num + 1
                #if num > 1000:
                    #break
                dT = (int(row[1]) - tLast) / 1000 / 1000
                tLast = int(row[1])
                F = np.matrix([[1, dT, 0.5 * dT * dT], [0, 1, dT], [0, 0, 1]]) # Physikmodell anpassen
                xHat = F * x # Predict
                PHat = F * P * F.T + Q
                # Messungsunwarscheinlichkeit erhöhen
                dM = xHat.item(0) - x.item(0)
                R.itemset((0, 0), R.item((0, 0)) + dM / (0.5 * dT * dT))
                R.itemset((1, 1), R.item((1, 1)) + dM)
                R.itemset((2, 2), R.item((2, 2)) + dM)
                # Zurücksetzen basierend auf Messung
                if row[2] == 'A':
                    R.itemset((0, 0), 0.35) # 0.35
                    z.itemset(0, float(row[5]))
                    a.append(z.item(0))
                elif row[2] == 'U':
                    R.itemset((1, 1), 0.005) # 0.005
                    z.itemset(1, float(row[3]) - 0.612217) # Offset still:0.7078 t1:0.315983 inc1:0.313751 t2:0.717831 t3:0.612217
                    u.append(z.item(1))
                elif row[2] == 'B':
                    R.itemset((2, 2), 1000.0)
                    z.itemset(2, float(row[3]) - 181.305481) # Offset still:991.864 t1:993.982361 inc1:994.080994 t2:992.712524 t3:181.305481
                    b.append(z.item(2))
                # Gain rechnen
                K = PHat * H.T * inv(H * PHat * H.T + R)
                P = (I - K * H) * PHat
                x = xHat + K * (z - H * xHat)
                #print(P)
                m.append(x.item(0))
        plt.plot(m)
        plt.ylim(-1.0, +2.0) 
        plt.show()
        #plt.plot(a)
        #plt.show()
        #plt.plot(u)
        #plt.show()
        #plt.plot(b)
        #plt.show()
        plt.matshow(P, cmap='binary')
        plt.show()
        plt.matshow(K, cmap='binary')
        plt.show()

main()