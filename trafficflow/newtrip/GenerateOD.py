from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
#import random
import time
#import networkx as nx
import numpy as np
#from numpy import ndarray
import gc
#from tensorflow.python.keras.models import load_model
#import json

RSNumber = 561
#rerouting_vehicle = {}

K = 7    #KSP
#con_threshold = 0.7 #congestion threshold value
#upstream_level = 3  #for selecting vehicles

# we need to import python modules from the $SUMO_HOME/tools directory
'''
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import sumolib
import traci  # noqa
'''
#total = 0
def loop(line,list1,list2,vehnum):
    list1_len = len(list1)
    list2_len = len(list2)
    
    total_len = list1_len * list2_len
    total_num = 0
    if vehnum % (total_len) != 0:
        #print(total_len)
        for n1 in list1:
            for n2 in list2:
                if n2 == list2[-1] and n1 == list1[-1]:
                    line += " "+str(n1)+" "+str(n2)+" "+str(int(vehnum -  total_num))+"\n"
                else:
                    line += " "+str(n1)+" "+str(n2)+" "+str(int(vehnum/total_len))+"\n"
                    total_num += int(vehnum/total_len)

    else:

        #print(total_len)
        for n1 in list1:
            for n2 in list2:
                line += " "+str(n1)+" "+str(n2)+" "+str(int(vehnum/total_len))+"\n"

    return line

# this is the main entry point of this script
if __name__ == "__main__":
    #if not os.path.exists('txt_result/out'+str(i)):
    #     os.makedirs('txt_result/out'+str(i))
    """
    left: 1~5    #13
    right:15~17  #7
    bottom:7~9   #9
    top:11~13    #10
    """
    line = """$OR;D2
* From-Time  To-Time
0.00 2.00
* Factor
1.00\n
    """
    # 1 ->  2 <-
    A= [ [1,3,5],[2,4,6],[15,17],[16,18],[7,9],[8,10],[11,13],[12,14]]

    totalnum = 22000

    leftnum = (totalnum / 2) / 20 *13
    rightnum = (totalnum / 2) / 20 *7
    topnum = (totalnum / 2) / 2
    bottomnum = (totalnum / 2) / 2
    line = loop(line,A[0],A[2],rightnum*0.3)# left1 to right 1
    line = loop(line,A[4],A[2],rightnum*0.3)# bottom1 to right 1
    line = loop(line,A[7],A[2],rightnum*0.3)# top2 to right1

    line = loop(line,A[3],A[2],rightnum*0.1)# right2 to right 1 



    line = loop(line,A[0],A[6],topnum*0.3)# left1 to top 1
    line = loop(line,A[4],A[6],topnum*0.3)# bottom1 to top 1
    line = loop(line,A[3],A[6],topnum*0.3)# right2 to top1

    line = loop(line,A[7],A[6],topnum*0.1)# top2 to top1


    line = loop(line,A[0],A[5],bottomnum*0.3)# left1 to bottom 2
    line = loop(line,A[3],A[5],bottomnum*0.3)# right2 to bottom2
    line = loop(line,A[7],A[5],bottomnum*0.3)# top2 to bottom2

    line = loop(line,A[4],A[5],bottomnum*0.1)# bottom1 to bottom 2

    line = loop(line,A[4],A[1],leftnum*0.3)# bottom1 to left 2
    line = loop(line,A[3],A[1],leftnum*0.3)# right2 to left 2
    line = loop(line,A[7],A[1],leftnum*0.3)# top2 to left2

    line = loop(line,A[0],A[1],leftnum*0.1)# left1 to left 2
    #print("--------------")
    #print(len(A[0]))
    #print(len(A[1]))

    
    wf = open("OD_file.od" , mode='w')    
    wf.write(line)
    wf.close()







