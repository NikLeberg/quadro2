#!/usr/bin/python3
# -*- coding: utf-8 -*
import sys
import csv
import matplotlib.pyplot as plt

def main():
    time = [0.0]
    armed = [0.0]
    throttle = [0.0]
    roll = [0.0]
    pitch = [0.0]
    x = [0.0]
    y = [0.0]
    K = [0.0]
    FL = [0.0]
    FR = [0.0]
    HL = [0.0]
    HR = [0.0]
    with open(str(sys.argv[1])) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=';')
        rows = iter(readCSV)
        row = next(rows)
        # Header aus erster Zeile extrahieren
        headArmed = row[1]
        headThrottle = row[2]
        headRoll = row[3]
        headPitch = row[4]
        headX = row[5]
        headY = row[6]
        headFL = row[7]
        headFR = row[8]
        headHL = row[9]
        headHR = row[10]
        row = next(rows)
        lastTimestamp = float(row[0])
        # über Zeilen iterieren
        while True:
            try:
                # Daten aus Zeile extrahieren
                # Time;pv/control/armed;parameter/control/throttle;pv/control/roll;pv/control/pitch;pv/control/xOut;pv/control/yOut
                row = next(rows)
                timestamp = float(row[0])
                dT = (timestamp - lastTimestamp) / 1000 # vergangene Zeit (in s)
                lastTimestamp = timestamp
                if (len(row) > 1 and row[1] != ''):
                    armed.append(float(row[1]))
                    throttle.append(throttle[-1])
                    roll.append(roll[-1])
                    pitch.append(pitch[-1])
                    x.append(x[-1])
                    y.append(y[-1])
                    FL.append(FL[-1])
                    FR.append(FR[-1])
                    HL.append(HL[-1])
                    HR.append(HR[-1])
                if (len(row) > 2 and row[2] != ''):
                    armed.append(armed[-1])
                    throttle.append(float(row[2]))
                    roll.append(roll[-1])
                    pitch.append(pitch[-1])
                    x.append(x[-1])
                    y.append(y[-1])
                    FL.append(FL[-1])
                    FR.append(FR[-1])
                    HL.append(HL[-1])
                    HR.append(HR[-1])
                if (len(row) > 3 and row[3] != ''):
                    armed.append(armed[-1])
                    throttle.append(throttle[-1])
                    roll.append(float(row[3]))
                    pitch.append(pitch[-1])
                    x.append(x[-1])
                    y.append(y[-1])
                    FL.append(FL[-1])
                    FR.append(FR[-1])
                    HL.append(HL[-1])
                    HR.append(HR[-1])
                if (len(row) > 4 and row[4] != ''):
                    armed.append(armed[-1])
                    throttle.append(throttle[-1])
                    roll.append(roll[-1])
                    pitch.append(float(row[4]))
                    x.append(x[-1])
                    y.append(y[-1])
                    FL.append(FL[-1])
                    FR.append(FR[-1])
                    HL.append(HL[-1])
                    HR.append(HR[-1])
                if (len(row) > 5 and row[5] != ''):
                    armed.append(armed[-1])
                    throttle.append(throttle[-1])
                    roll.append(roll[-1])
                    pitch.append(pitch[-1])
                    x.append(float(row[5]))
                    y.append(y[-1])
                    FL.append(FL[-1])
                    FR.append(FR[-1])
                    HL.append(HL[-1])
                    HR.append(HR[-1])
                if (len(row) > 6 and row[6] != ''):
                    armed.append(armed[-1])
                    throttle.append(throttle[-1])
                    roll.append(roll[-1])
                    pitch.append(pitch[-1])
                    x.append(x[-1])
                    y.append(float(row[6]))
                    FL.append(FL[-1])
                    FR.append(FR[-1])
                    HL.append(HL[-1])
                    HR.append(HR[-1])
                if (len(row) > 7 and row[7] != ''):
                    armed.append(armed[-1])
                    throttle.append(throttle[-1])
                    roll.append(roll[-1])
                    pitch.append(pitch[-1])
                    x.append(x[-1])
                    y.append(y[-1])
                    FL.append(float(row[7]))
                    FR.append(FR[-1])
                    HL.append(HL[-1])
                    HR.append(HR[-1])
                if (len(row) > 8 and row[8] != ''):
                    armed.append(armed[-1])
                    throttle.append(throttle[-1])
                    roll.append(roll[-1])
                    pitch.append(pitch[-1])
                    x.append(x[-1])
                    y.append(y[-1])
                    FL.append(FL[-1])
                    FR.append(float(row[8]))
                    HL.append(HL[-1])
                    HR.append(HR[-1])
                if (len(row) > 9 and row[9] != ''):
                    armed.append(armed[-1])
                    throttle.append(throttle[-1])
                    roll.append(roll[-1])
                    pitch.append(pitch[-1])
                    x.append(x[-1])
                    y.append(y[-1])
                    FL.append(FL[-1])
                    FR.append(FR[-1])
                    HL.append(float(row[9]))
                    HR.append(HR[-1])
                if (len(row) > 10 and row[10] != ''):
                    armed.append(armed[-1])
                    throttle.append(throttle[-1])
                    roll.append(roll[-1])
                    pitch.append(pitch[-1])
                    x.append(x[-1])
                    y.append(y[-1])
                    FL.append(FL[-1])
                    FR.append(FR[-1])
                    HL.append(HL[-1])
                    HR.append(float(row[10]))
                # speichere für Plot
                time.append(time[-1] + dT)

            except (StopIteration) as e:
                print(e)
                break

        # Resultat
        plt.title('PID')
        plt.plot(time, armed, label=headArmed)
        plt.plot(time, throttle, label=headThrottle)
        plt.plot(time, roll, label=headRoll)
        plt.plot(time, pitch, label=headPitch)
        plt.plot(time, x, label=headX)
        plt.plot(time, y, label=headY)
        plt.plot(time, FL, label=headFL)
        plt.plot(time, FR, label=headFR)
        plt.plot(time, HL, label=headHL)
        plt.plot(time, HR, label=headHR)
        #plt.plot(time, K, label="K")
        plt.legend()
        plt.show()

main()
exit()
