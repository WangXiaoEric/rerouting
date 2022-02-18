from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
import csv
import math
from random import sample
import pandas as pd
import math
import xml.etree.cElementTree as ET
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import sumolib





if __name__ == "__main__":
    tree = ET.ElementTree(file='data/RSUsLocationUBC.xml')
    tree.getroot()
    Zoneidx = 0
    for elem in tree.iter(tag='poly'):
        Zoneidx += 1
        continue
    for i in range(30):
        if not os.path.exists('entropycsv_result/'):
            os.makedirs('entropycsv_result/')
        data = pd.DataFrame(columns=["Ev"])
        try:
                with open("entropytxt_result/entropy"+str(i)+".txt",'r') as rf:
                    for line in rf:
                        line.replace("\n", "")
                        if line != "":
                            token = line.split(",")
                            #- summation(pi/N*ln(pi/N))
                            N = float(token[Zoneidx])
                            totalNum = 0
                            for tokeni in range(Zoneidx):
                                if float(token[tokeni]) != 0:
                                    totalNum += (float(token[tokeni]) / N ) * math.log( float(token[tokeni]) / N  ) 
                                else:
                                    totalNum += 0

                            totalNum = -totalNum
                            newrow = {"Ev":totalNum}
                            data = data.append(newrow, ignore_index=True)
                    data.to_csv('entropycsv_result/out'+str(i)+".csv" , index=False)

        except:
            continue
