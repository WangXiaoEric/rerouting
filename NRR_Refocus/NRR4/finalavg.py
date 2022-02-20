from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
import csv
import math
from random import sample
import pandas as pd
import numpy as np
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import sumolib
def AverageAllData(inputdir,outputdir,lane,filenum):

    total = 0
    flag =0
    for outeridx in range(filenum):
        try:
            tempAvg_data = pd.read_csv(inputdir+str(outeridx)+"/"+lane+".csv")
            total += 1
        except:
            #print("Average File "+lane+".csv not exist")
            continue
        if flag ==0:
            
            avg_data = tempAvg_data
            avg_intervel= tempAvg_data["interval"]
            flag =1
            #print(len(avg_data))
        else:
            if len(avg_data) >= len(tempAvg_data):
                avg_data += tempAvg_data.reindex_like(avg_data).fillna(13.89)
            else:
                avg_data = avg_data.reindex_like(tempAvg_data).fillna(13.89) + tempAvg_data
                avg_intervel = tempAvg_data["interval"]

        #count_mean = np.isinf(avg_data).values.sum()
        #if  count_mean>0:
        #    print(outeridx)
    #a = avg_data["avgspeed"].tolist()

    #for i,b in enumerate(a):
    #    print("i= ",i)
    #    print(b)

    if total==0:
        return
    avg_data /= total
    avg_data["interval"] = avg_intervel
    avg_data.to_csv(outputdir+"/"+lane+"_mean.csv", index=False)


if __name__ == "__main__":
    NetName = "data/01/Tainan.net.xml"
    net = sumolib.net.readNet(NetName)
    edges = net.getEdges()
    Lane_list = []
    #'''
    for edge in edges:
        lanes = edge.getLanes()
        for lane in lanes:
            Lane_list.append(lane.getID())
            #Lane_dict[lane.getID()] = lane.getLength()
    #'''
    #Lane_list.append("319713269#3_1")
    #Lane_dict[] = 5
    inputdir = 'interavg_result/out'
    outputdir = 'finalavg_result/'
    for lane in Lane_list:
        AverageAllData(inputdir,outputdir,lane,5)

