#!/usr/bin/python
# -*- coding: utf-8 -*
import sys
import csv


def main():

    s = ''
    l = []

    with open(str(sys.argv[1])) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            c = row[1]
            if 'CR' in c:
                s = ''.join(l)
                l = []
            elif 'LF' in c:
                print(s)
            elif 'SP' in c:
                l.append(' ')
            elif len(row) == 5:
                l.append(',')
            else:
                l.append(c)


main()