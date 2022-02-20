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
    Lane_dict = {}
    #'''
    for edge in edges:
        lanes = edge.getLanes()
        for lane in lanes:
            Lane_dict[lane.getID()] = lane.getLength()
    #'''
    #Lane_dict["-111343195#2_0"] = 5
    for i in range(0,1):
        if not os.path.exists('csv_result/out'+str(i)):
            os.makedirs('csv_result/out'+str(i))

        for lane, lanelen in Lane_dict.items():
            data = pd.DataFrame(columns=["entry","exit","length"])
            lane_len = lanelen
            try:
                with open("txt_result/out"+str(i)+"/"+lane+".txt",'r') as rf:
                    for line in rf:
                        line.replace("\n", "")
                        if line != "":
                            token = line.split(",")
                            veh_entry = float(token[1])
                            veh_exit = float(token[2])
                            if veh_entry <= veh_exit:
                              newrow = {"entry":veh_entry ,"exit": veh_exit,"length":lane_len}
                            else:
                              
                              newrow = {"entry":veh_exit ,"exit": veh_entry,"length":lane_len}
                            data = data.append(newrow, ignore_index=True)
                    #print(data["exit"])
                    data = data.sort_values(by=["exit"])
                    #print("enter")
                    #print(data["exit"])
                    data.to_csv('csv_result/out'+str(i)+"/"+lane +".csv" , index=False)

            except:
                continue

