from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
#import random
import time
#import networkx as nx
import numpy as np
import subprocess
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

# this is the main entry point of this script
if __name__ == "__main__":
    #os.system("python GenerateOD.py")
    #os.system("od2trips -c od2trips.config.xml -n taz_file.taz.xml -d OD_file.od -o od_file.odtrips.xml --random true")
    #os.system("duarouter –c duarcfg_file.trips2routes.duarcfg -o od_route_file.odtrips1.rou.xml")
    #subprocess.run(["duarouter", "-c", "duarcfg_file.trips2routes.duarcfg","-o","od_route_file.odtrips1.rou.xml"])
    #os.system("duarouter –c duarcfg_file.trips2routes.duarcfg -o od_route_file.odtrips"+str(1+1)+".rou.xml")

    
    for i in range(15):
        print("i= ",i+1)
        os.system("od2trips -c od2trips.config.xml -n taz_file.taz.xml -d OD_file.od -o od_file.odtrips.xml --random true")
        subprocess.run(["duarouter", "-c", "duarcfg_file.trips2routes.duarcfg","-o","od_route_file.odtrips"+str(i+1)+".rou.xml"])
        #os.system("duarouter –c duarcfg_file.trips2routes.duarcfg -o od_route_file.odtrips"+str(i+1)+".rou.xml")
        #time.sleep(5)
    os.system("del *.alt.xml")





