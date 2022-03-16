#COST2與COST1的不同
# 1、 COST1只用當前當下速度  COST2是可以用LSTM，再把瞬時速度轉為每一輛車的平均速度
# 2、 COST1沒有DB  COST2有DB
# 3、 COST1沒有用LSTM COST2用到LSTM（並且是預測的是error）


# Attention:必須把Cost1 run完畢（睦昆建議可以先跑10次），才能run2.
# COST2run完，會產生檔案，然後用新檔案再run一次COst2

from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
from optparse import OptionParser
#import random
import time
import datetime
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
os.environ["CUDA_VISIBLE_DEVICES"] = '-1'
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
    real_time_mid_loop_detector_speed_info = {}
    tensec_avgspeed_info = {}
    five_minu_loopd_avg_speed_result = {}
    five_minu_avgspeed_result = {}
    edge_occ_dict = {}
    oneminspeed_dict = {}
    for EdgeID in alledgelist:
        
        if EdgeID.find(':') == -1:
            real_time_mid_loop_detector_speed_info[EdgeID] = []
            tensec_avgspeed_info[EdgeID] = []
            oneminspeed_dict[EdgeID] = []
            five_minu_loopd_avg_speed_result[EdgeID] = []
            edge_occ_dict[EdgeID] = {}



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


        

    return Lane_dict , real_time_mid_loop_detector_speed_info , oneminspeed_dict , five_minu_loopd_avg_speed_result , edge_occ_dict , Zone_dict , EdgeTOZone_dict , tensec_avgspeed_info , five_minu_avgspeed_result


def run(i,VR):
    total_diff = 0
    total_same = 0
    log = ""
    #s = 0
    step = 0

    #Lane_dict - 道路loop检测器对象
    #speed_info - 以edgeID为字典的数据
    Lane_dict , real_time_mid_loop_detector_speed_info , oneminspeed_dict , five_minu_loopd_avg_speed_result , edge_occ_dict , \
    Zone_dict , EdgeTOZone_dict , tensec_avgspeed_info , five_minu_avgspeed_result = initialize()

    """get road network info."""    
    RS_info, con_info, TL_info, conn_TL = gni.getRNinfo()
    Road_Network = gni.loadedgeRN(RS_info, con_info)
    Node_Coordinate = gni.getCoordinate()
    NetName = "data/01/Tainan.net.xml"
    net = sumolib.net.readNet(NetName)
    if not os.path.exists('CVtxt_result/'):
        os.makedirs('CVtxt_result/')
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        #每一步骤都需要记录状况，与记录日志的道理是一致的
        Lane_dict = GetStart(Lane_dict)
        Lane_dict = GetMid(Lane_dict)
        Lane_dict = GetEnd(Lane_dict)
        #每一步骤记录Mid_ detector的速度信息  后面需要关注一下speed_info的速度情况；
        for EdgeID, speed_list in real_time_mid_loop_detector_speed_info.items():
            count = 0
            instspeed = 0
            total_speed = 0
            for laneid in range(net.getEdge(EdgeID).getLaneNumber()):
                loopid ="Mid_"+EdgeID+"_"+str(laneid)
                instspeed = traci.inductionloop.getLastStepMeanSpeed(loopid)
                if instspeed == -1:
                    continue
                else:
                    total_speed += instspeed
                    count += 1
            if count != 0:
                speed_list.append(total_speed/count)

        SimTime = int(traci.simulation.getTime())
        #if SimTime > 400:
        #    break
        """get speed info: do every 10 sec. (time step)"""
        #'''
        if SimTime  % 10 == 0:
            for EdgeID, speed_list in tensec_avgspeed_info.items():
                speed_list.append(traci.edge.getLastStepMeanSpeed(EdgeID))
                
        if SimTime %60==0:
            #更新完毕之后 tensec_avgspeed_info这个数据要清空
            """ update speed data per min."""
            for EdgeID, speed_list in tensec_avgspeed_info.items():
                temp_list = speed_list
                tempAvgSpeed = np.mean(temp_list)
                oneminspeed_dict[EdgeID].append(tempAvgSpeed)
                speed_list[:] = []
        #'''
        if SimTime %300 == 0:
        # if SimTime %100 == 0:
            #''' 执行完毕之后一分钟字典oneminspeed_dict清空
            for EdgeID, speed_list in oneminspeed_dict.items():
                if len(speed_list) !=5:
                    print("speed list !=5")
                    exit(0)
                temp_list = speed_list
                tempAvgSpeed = np.mean(temp_list)
                five_minu_avgspeed_result[EdgeID] = tempAvgSpeed
                speed_list[:] = []
            #'''

            for EdgeID, speed_list in real_time_mid_loop_detector_speed_info.items():
                temp_list = speed_list
                if not temp_list:
                    if five_minu_avgspeed_result[EdgeID] >= 11:
                        five_minu_loopd_avg_speed_result[EdgeID].append(13.89)
                    else:
                        five_minu_loopd_avg_speed_result[EdgeID].append(0)
                else:
                    tempAvgSpeed = np.mean(temp_list)
                    five_minu_loopd_avg_speed_result[EdgeID].append(tempAvgSpeed)
                speed_list[:] = []
                
            diff = 0
            same = 0
            print("SimeTime: ",SimTime)
            """detect congestion per 5 min."""
            log += "SUMO Time: "+str(SimTime)

            time_start = time.time()
            #获取上一step(即当前)道路车辆数量
            RSDensities = tcp.updateRSDensities(RS_info)
            congestedRS, occ_vel_content, edge_occ_dict = tcp.detectCongestion(RSDensities, con_threshold, edge_occ_dict)
            time_detect = time.time()
            log += " ,detect time: "+str( time_detect-time_start )

            #tcp.log_entropy(EdgeTOZone_dict,i,len(Zone_dict),RSDensities)
            #print("congestedRS",congestedRS)
            
                       
            if len(congestedRS) > 0:
                vehicleRS_dict = {}
                selected_vehicles = []
                log += " ,Congestion Road and upstream_level: "
                for rs_edge_id in congestedRS:

                    upstream_level = round(np.exp(5.37*congestedRS[rs_edge_id]-3.07))
                    #print("upstream_level= ",upstream_level)
                    log +=  rs_edge_id+"+"+ str(upstream_level)+" "
                    RS_from = RS_info[rs_edge_id][0]
                    RS_to = RS_info[rs_edge_id][1]
 
                    """select affected vehicles 根据每个路段找到Up stream level"""
                    level_dictionary = vs.getAffectedRSviaedge(rs_edge_id, Road_Network, upstream_level, RS_info, Node_Coordinate)
                    #找到每一个路段所对应的
                    veh_list , vehicleRS_dict = vs.selectVehicles(rs_edge_id, level_dictionary, vehicleRS_dict, RS_from, RS_to)
                    selected_vehicles += veh_list 

                #delete duplicated element
                selected_vehicles = sorted(set(selected_vehicles), key = selected_vehicles.index)
                #找到车辆最近的拥塞路段
                vehicleRS_dict = vr.ChooseNearCongRoad(selected_vehicles, vehicleRS_dict, RS_info, Node_Coordinate)
                """"rank the vehicles"""
                if VR == "ld":
                    ranked_vehicles = vr.rankVehicles(selected_vehicles, RS_info)
                    
                Road_Network = ra.SetGraphWeight(Road_Network , RSDensities , con_info, Zone_dict)
                ODpairs = ra.getODpairs(ranked_vehicles)
                paths = ra.getAllKSP(Road_Network, ODpairs, K)
                all_path_RS_list = ra.getAllRS(paths)
                
                """"start re-routing""" 
                diff, same = ra.Reroute(RS_info, TL_info, conn_TL, paths, five_minu_loopd_avg_speed_result, ranked_vehicles,
                                         Node_Coordinate, RSDensities, vehicleRS_dict, diff, same, all_path_RS_list, None, MeanZ_dict, Model_dict)
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
        wf = open("txt_result/out"+str(i)+"/"+laneid+".txt" , mode='w') #mode='w' 或者 'a'
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

    for edgeid in edge_occ_dict.keys():
        wf = open("occtxt_result/out"+str(i)+"/"+edgeid+".txt" , mode='w')    
        for Simtime,info in edge_occ_dict[edgeid].items():
            line = str(Simtime)+","+info+"\n"
            wf.write(line)
        wf.close()

    #記錄最終的SimTime,用來補充數據
    summary = open("txt_result/out" + str(i) + "/summary.txt", mode='w')
    print("SimTime is " + str(SimTime))
    summary.write(str(SimTime))
    summary.close()

    gc.collect()
    


# this is the main entry point of this script
if __name__ == "__main__":

    #先清理所有文件，除过LSTM MODEL 文件
    os.system("python clean_all_files.py")

    parser = OptionParser()#parser = OptionParser()
    # python3 main - s 0 - e 45
    # self loop. we want to run only once case, and train Driver-Behavour and LSTM
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

    """get history average speed""" #从mean文件夹中获取资料， 需要考虑如何在Mean中生成资料 TODO 此部分数据可以删除掉
    # MeanSpeed_dict = gni.GetPastMeanSpeed()

    """get history Zmax Zmin""" #从meanZ文件夹中获取资料， 需要考虑如何在meanZ中生成资料 TODO 此部分数据需要热启动
    MeanZ_dict = gni.GetPastMeanZ()
    """get model for all road""" #获取所有路段的深度训练模型  TODO checkmodel是否全部读出

    print("Before Deep Learning model load Time:" + str(datetime.datetime.now()))
    Model_dict = gni.GetRoadModel()
    print("After Deep Learning model load Time:" + str(datetime.datetime.now()))

    #为了生成热启动数据 TODO
    # Model_dict = None

    
    for i in range(start,end):

        pn = "data/01/re01.sumocfg"

        #每次输出都用不同的文件
        fn = "Experiment_result/tripinfo"+str(i+1)+".xml"

        print(i+1,"begin at:",time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
        traci.start([sumoBinary, "-c", pn,"--tripinfo-output", fn])
        run(i,"ld")
        print("finish at:",time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
        gc.collect()
    gc.collect()

    os.system("python txt_to_csv.py -s "+str(start)+" -e "+str(end))
    # 生成interavg_result 与interZ_result
    os.system("python veh_avg_inst_lstm.py -s "+str(start)+" -e "+str(end))
    if start == 0:
        #  生成finalZ_result
        os.system("python finalavg.py")
        #  finalZ_result 重新命名為 meanZ 放入data 資料夾  TODO out0 同时要将热启动数据移动到最新地址 out0_init；
        os.system("python move_mean_files.py")
        # os.system("python3 train_lstm.py")







