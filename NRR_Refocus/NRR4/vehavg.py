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
def CalMinsAvgSpeedSingleFile(outputdir , data ,LoopTime = 60):
    outputdir = outputdir 
    out = pd.DataFrame(columns=["interval","avgspeed"])
    speedfactor = 1#0.95
    #print(len(data))
    for idx in range(len(data)):
        #print(idx)
        if idx == 0:
            t =  int(data.iloc[idx]["time"]/LoopTime) # calculate t every mins
            StartIdx = 0 # for pandas csv 
            if t !=0 : #not start from  0 
                for i in range(t):
                    newrow = {"interval":i ,"avgspeed": speedfactor * 13.89}
                    out = out.append(newrow, ignore_index=True)
                #print(len(out))
                #print("------")

        if t != int(data.iloc[idx]["time"]/LoopTime): #arrive next interval
                EndIdx = idx
                num = len(data[StartIdx:EndIdx]["avgspeed"])
                if num == 0:
                    avg_speed = 0
                else :
                    avg_speed = data[StartIdx:EndIdx]["avgspeed"].sum()/num

                interval = t
                newrow = {"interval":interval ,"avgspeed": avg_speed }
                out = out.append(newrow, ignore_index=True)
                StartIdx = idx # StartIdx = EndIdx
                #print(len(out))
                #print("up----")
                if int(data.iloc[idx]["time"]/LoopTime) != t+1: #next interval not continous, compliment 0
                    #print(int(data.iloc[idx]["time"]/LoopTime) - t)
                    for i in range(1,int(data.iloc[idx]["time"]/LoopTime) - t):
                        if out.tail(1)["avgspeed"].iloc[0] < speedfactor * 13.89/2:
                            newrow = {"interval":interval + i ,"avgspeed": 0 }
                        else:
                            newrow = {"interval":interval + i ,"avgspeed": speedfactor * 13.89 }
                        out = out.append(newrow, ignore_index=True)
                        #print(len(out))
                        #print("down------")

                t = int(data.iloc[idx]["time"]/LoopTime) # update t
    #print(len(out))
    #leave loop
    EndIdx = idx + 1 #last one need to plus 1
    num = len(data[StartIdx:EndIdx]["avgspeed"])
    if num == 0:
        avg_speed = 0
    else :
        avg_speed = data[StartIdx:EndIdx]["avgspeed"].sum()/num

    interval = t 

    newrow = {"interval":interval ,"avgspeed": avg_speed }
    out = out.append(newrow, ignore_index=True)
    out.to_csv(outputdir, index=False)


if __name__ == "__main__":
    NetName = "data/01/Tainan.net.xml"
    net = sumolib.net.readNet(NetName)
    edges = net.getEdges()
    Lane_list = []
    
    for edge in edges:
        lanes = edge.getLanes()
        for lane in lanes:
            Lane_list.append(lane.getID())
            #Lane_dict[lane.getID()] = lane.getLength()
    
    #Lane_list.append("160253726_1")
    #Lane_dict[] = 5
    speedFactor = 1.5
    for i in range(0,1):
        print("i= ",i)
        if not os.path.exists('interavg_result/out'+str(i)):
            os.makedirs('interavg_result/out'+str(i))

        for lane in Lane_list:
            try:
                data = pd.read_csv("csv_result/out"+str(i)+"/"+lane+".csv")
                temp = pd.DataFrame(columns=["time","avgspeed"])
                temp["avgspeed"] = data["length"] / (data["exit"] - data["entry"]).abs()
                temp["time"] = data["exit"]
                temp = temp[~temp.isin([np.nan, np.inf, -np.inf]).any(1)]
                a = np.array(temp["avgspeed"].values.tolist())

                temp["avgspeed"] = np.where(a > 13.89*speedFactor, 13.89*speedFactor, a).tolist()

                #temp.to_csv("123.csv" , index=False)
                CalMinsAvgSpeedSingleFile('interavg_result/out'+str(i)+"/"+lane +".csv" , temp ,300)
            except Exception as e:
                #print(e)
                continue


