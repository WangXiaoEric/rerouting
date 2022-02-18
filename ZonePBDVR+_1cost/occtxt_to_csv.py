#處理數據
from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
import csv
import math
from random import sample
import pandas as pd
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import sumolib





if __name__ == "__main__":
    NetName = "data/01/Tainan.net.xml"
    net = sumolib.net.readNet(NetName)
    edges = net.getEdges()

    for i in range(0,1):
        if not os.path.exists('occcsv_result/out'+str(i)):
            os.makedirs('occcsv_result/out'+str(i))
            

        for edge in edges:
            data = pd.DataFrame(columns=["occ"])

            try:
                with open("occtxt_result/out"+str(i)+"/"+edge.getID()+".txt",'r') as rf:
                    for line in rf:
                        line.replace("\n", "")
                        if line != "":
                            token = line.split(",")
                            occ = float(token[1])
                            if occ >1:
                                occ = 1
                            newrow = {"occ":occ }
                            data = data.append(newrow, ignore_index=True)
                    #print(data["exit"])
                    #data = data.sort_values(by=["exit"])
                    #print("enter")
                    #print(data["exit"])
                    data.to_csv('occcsv_result/out'+str(i)+"/"+edge.getID() +".csv" , index=False)

            except Exception as e:
                print(e)
                continue

