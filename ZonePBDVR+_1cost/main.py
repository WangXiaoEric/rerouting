from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
from optparse import OptionParser
#import random
import time
#import networkx as nx
import numpy as np
#from numpy import ndarray
import gc
import get_network_info as gni
import traffic_congestion_prediction as tcp
import vehicle_selection as vs
import vehicle_rank as vr
import reroute_algorithm as ra
import xml.etree.cElementTree as ET
#from tensorflow.python.keras.models import load_model
#import json

RSNumber = 561
#rerouting_vehicle = {}

K = 7    #KSP
#con_threshold = 0.7 #congestion threshold value
#upstream_level = 3  #for selecting vehicles

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import sumolib
import traci  # noqa

#車輛碰到感應器是在時間中的第幾秒 道路起始段
def GetStart(Lane_dict):
    for laneid in Lane_dict.keys():

      loopid ="Start_"+laneid
      info = traci.inductionloop.getVehicleData(loopid)
      if  info : # not empty list
          veh_id = info[0][0]
          veh_entry = info[0][2]
          if veh_id not in Lane_dict[laneid]:
              Lane_dict[laneid][veh_id] = {}
          Lane_dict[laneid][veh_id]["veh_entry"] = veh_entry

    return Lane_dict

#車輛碰到感應器是在時間中的第幾秒，以及通過當前瞬時速度
def GetMid(Lane_dict):
    for laneid in Lane_dict.keys():

      loopid ="Mid_"+laneid
      info = traci.inductionloop.getVehicleData(loopid)
      if  info : # not empty list
          if -1 == info[0][3]:
              continue
          veh_id = info[0][0]
          veh_entry = info[0][2]
          if veh_id not in Lane_dict[laneid]:
              continue
          instspeed = traci.inductionloop.getLastStepMeanSpeed(loopid)
          if instspeed == -1:
              continue
          Lane_dict[laneid][veh_id]["veh_midentry"] = veh_entry
          Lane_dict[laneid][veh_id]["veh_mid"] = instspeed


    return Lane_dict

#車輛碰到感應器是在時間中的第幾秒 道路終點
def GetEnd(Lane_dict):
    for laneid in Lane_dict.keys():

      loopid ="End_"+laneid
      info = traci.inductionloop.getVehicleData(loopid)
      if  info : # not empty list
          if -1 == info[0][3]:
              continue
          veh_id = info[0][0]
          veh_exit = info[0][3]
          if veh_id not in Lane_dict[laneid]:
              Lane_dict[laneid][veh_id] = {}
          Lane_dict[laneid][veh_id]["veh_exit"] = veh_exit

    return Lane_dict

#基本參數初始化
def initialize():
    NetName = "data/01/Tainan.net.xml"
    net = sumolib.net.readNet(NetName)
    edges = net.getEdges()
    Lane_dict = {}
    for edge in edges:
        lanes = edge.getLanes()
        for lane in lanes:
            Lane_dict[lane.getID()] = {}

    alledgelist = traci.edge.getIDList()
    speed_info = {}
    speed_result = {}
    edgeocc_dict = {}
    oneminspeed_dict = {}
    for EdgeID in alledgelist:

        if EdgeID.find(':') == -1:
            speed_info[EdgeID] = []
            oneminspeed_dict[EdgeID] = []
            speed_result[EdgeID] = []
            edgeocc_dict[EdgeID] = {}



    tree = ET.ElementTree(file='data/RSUsLocationUBC.xml')
    tree.getroot()

    Zone_dict = {}
    Zoneidx = 0
    AllEdge_list = []
    EdgeTOZone_dict = {}
    for elem in tree.iter(tag='poly'):
        x , y = elem.attrib['center'].split(",")
        edges = net.getNeighboringEdges(float(x), float(y), 330)#330
        ZoneEdge_list = []
        for edge in edges:
            closestEdge , distance = edge
            #print(closestEdge)
            EdgeID = str(closestEdge).split('id=')[1].split(' ')[0].replace('"',"")
            #if EdgeID == "319713269#2":
            #    print("Yes")
            #print(EdgeID)
            if EdgeID in AllEdge_list:
                continue
            EdgeTOZone_dict[EdgeID] = Zoneidx
            AllEdge_list.append(EdgeID)

            ZoneEdge_list.append(EdgeID)

        Zone_dict[Zoneidx] = ZoneEdge_list

        Zoneidx += 1

    return Lane_dict , speed_info , oneminspeed_dict , speed_result , edgeocc_dict , Zone_dict ,EdgeTOZone_dict

#模擬sumo
def run(i,VR):
    total_diff = 0
    total_same = 0
    log = ""
    #s = 0
    step = 0

    Lane_dict , speed_info , oneminspeed_dict , speed_result , edgeocc_dict , Zone_dict , EdgeTOZone_dict = initialize()

    """get road network info."""
    RS_info, con_info, TL_info, conn_TL = gni.getRNinfo()
    RN = gni.loadedgeRN(RS_info, con_info)
    # Node info
    Coordinate = gni.getCoordinate()
    if not os.path.exists('CVtxt_result/'):
        os.makedirs('CVtxt_result/')

    while traci.simulation.getMinExpectedNumber() > 0:
        if step == 99:
            print("=======")
        traci.simulationStep()

        """collect road speed and output in txt_result/"""
        Lane_dict = GetStart(Lane_dict)
        Lane_dict = GetMid(Lane_dict)
        Lane_dict = GetEnd(Lane_dict)

        SimTime = int(traci.simulation.getTime())
        #if SimTime > 400:
        #    break
        """get speed info: do every 10 sec. (time step)"""
        if SimTime  % 10 == 0:
            for EdgeID, speed_list in speed_info.items():
                speed_list.append(traci.edge.getLastStepMeanSpeed(EdgeID))

        if SimTime %60==0:

            """ update speed data per min."""
            for EdgeID, speed_list in speed_info.items():
                temp_list = speed_list
                tempAvgSpeed = np.mean(temp_list)
                oneminspeed_dict[EdgeID].append(tempAvgSpeed)
                speed_list[:] = []

        if SimTime %300 == 0:

            for EdgeID, speed_list in oneminspeed_dict.items():
                if len(speed_list) !=5:
                    print("speed list !=5")
                    exit(0)
                temp_list = speed_list
                tempAvgSpeed = np.mean(temp_list)
                speed_result[EdgeID] = tempAvgSpeed
                speed_list[:] = []

            diff = 0
            same = 0
            print("SimeTime: ",SimTime)
            """detect congestion per 5 min."""
            log += "SUMO Time: "+str(SimTime)

            time_start = time.time()
            RSDensities = tcp.updateRSDensities(RS_info, RSNumber)
            congestedRS, content,edgeocc_dict = tcp.detectCongestion(RSDensities, con_threshold, edgeocc_dict)
            time_detect = time.time()
            log += " ,detect time: "+str( time_detect-time_start )

            #tcp.log_entropy(EdgeTOZone_dict,i,len(Zone_dict),RSDensities)
            #print("congestedRS",congestedRS)


            if len(congestedRS) > 0:
                vehicleRS_dict = {}
                selected_vehicles = []
                log += " ,Congestion Road and upstream_level: "
                for RS in congestedRS:
                    """Set upstream level """
                    upstream_level = round(np.exp(5.37*congestedRS[RS]-3.07))
                    #print("upstream_level= ",upstream_level)
                    log +=  RS+"+"+ str(upstream_level)+" "
                    RS_from = RS_info[RS][0]
                    RS_to = RS_info[RS][1]

                    """select affected vehicles"""
                    level_dictionary = vs.getAffectedRSviaedge(RS, RN, upstream_level, RS_info, Coordinate)
                    veh_list , vehicleRS_dict = vs.selectVehicles(RS, level_dictionary, vehicleRS_dict, RS_from, RS_to)
                    selected_vehicles += veh_list

                #delete duplicated element
                selected_vehicles = sorted(set(selected_vehicles), key = selected_vehicles.index)
                """choose the nearest congest road for vehicle"""
                vehicleRS_dict = vr.ChooseNearCongRoad(selected_vehicles, vehicleRS_dict, RS_info, Coordinate)
                """"rank the vehicles"""
                if VR == "ld":
                    ranked_vehicles = vr.rankVehicles(selected_vehicles, RS_info)
                """set New graph weight"""
                RN = ra.SetGraphWeight(RN , RSDensities , con_info,Zone_dict)
                ODpairs = ra.getODpairs(ranked_vehicles)
                paths = ra.getAllKSP(RN, ODpairs, K)
                RS_list = ra.getAllRS(paths)

                """"start re-routing"""
                diff , same = ra.Reroute(RS_info, TL_info, conn_TL, paths, speed_result, ranked_vehicles, Coordinate, RSDensities, vehicleRS_dict, diff, same, RS_list)
                time_end = time.time()
                log += ",Reroute time: "+str( time_end - time_detect)
                gc.collect()
            #break
            gc.collect()
            step = 0
            log += " ,NewPath diff: "+str(diff) + " ,NewPath same: " +str(same) + " ,NewPath reroute: "+str(diff+same)+"\n"
            total_diff += diff
            total_same += same




        step += 1
        #s += 1
    traci.close()
    sys.stdout.flush()
    if not os.path.exists('log_result/'):
        os.makedirs('log_result/')
    wf = open("log_result/log"+str(i)+".txt" , mode='w')
    log += "total NewPath diff: "+str(total_diff)+" ,total NewPath same: "+str(total_same)+" ,total NewPath reroute: "+str(total_diff + total_same)
    wf.write(log)
    wf.close()

    if not os.path.exists('txt_result/out'+str(i)):
        os.makedirs('txt_result/out'+str(i))

    for laneid in Lane_dict.keys():
        if len(Lane_dict[laneid]) == 0:
            continue
        wf = open("txt_result/out"+str(i)+"/"+laneid+".txt" , mode='w')
        for veh_id in Lane_dict[laneid]:
            try:
              line = str(veh_id)+","+str(Lane_dict[laneid][veh_id]["veh_entry"])+","+str(Lane_dict[laneid][veh_id]["veh_midentry"])+","+\
              str(Lane_dict[laneid][veh_id]["veh_mid"])+","+str(Lane_dict[laneid][veh_id]["veh_exit"])+"\n"
              wf.write(line)
            except:
                continue
        wf.close()

    if not os.path.exists('occtxt_result/out'+str(i)):
        os.makedirs('occtxt_result/out'+str(i))

    for edgeid in edgeocc_dict.keys():
        wf = open("occtxt_result/out"+str(i)+"/"+edgeid+".txt" , mode='w')
        for Simtime,info in edgeocc_dict[edgeid].items():
            line = str(Simtime)+","+info+"\n"
            wf.write(line)
        wf.close()

    gc.collect()


# this is the main entry point of this script
if __name__ == "__main__":
    parser = OptionParser()#parser = OptionParser()
    parser.add_option("-s", "--start", dest="start",type="int" , default="0", help="The start used to run SUMO start from tripinfox.xml")
    parser.add_option("-e", "--end", dest="end",type="int" , default="1", help="The end used to run SUMO end at tripinfox.xml")
    parser.add_option("--nogui", action="store_true",default=False, help="run the commandline version of sumo")
    (options, args) = parser.parse_args()
    start = options.start
    end = options.end

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo')

    con_threshold = 0.7 #congestion threshold value


    for i in range(start,end):
        # 配置sumo配置文件
        if i < 9:
            pn = "data/01"+"/re0"+str(i+1)+".sumocfg"
            fn = "Experiment_result/tripinfo"+str(i+1)+".xml"
        else:
            pn = "data/01"+"/re"+str(i+1)+".sumocfg"
            fn = "Experiment_result/tripinfo"+str(i+1)+".xml"

        print(i+1,"begin at:",time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
        traci.start([sumoBinary, "-c", pn,"--tripinfo-output", fn])
        run(i,"ld")
        print("finish at:",time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
        gc.collect()
    gc.collect()
    os.system("python3 txt_to_csv.py -s "+str(start)+" -e "+str(end))
    os.system("python3 vehavg_inst.py -s "+str(start)+" -e "+str(end))
    if start == 0:
        os.system("python3 finalavg.py")
        os.system("python3 train.py")







