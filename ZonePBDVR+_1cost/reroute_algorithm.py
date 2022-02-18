# -*- coding: utf-8 -*-
import traci
import numpy as np
import networkx as nx
from itertools import islice
import random

import gc
import pick_path as pp

"""do Reroute"""
def Reroute(RS_info, TL_info, conn_TL, paths, speed_result, ranked_vehicles, Coordinate, RSDensities, vehicleRS_dict, diff, same, RS_list):
    
    predicted_speeds = {}
    for RS in RS_list:
        predicted_speeds[RS] = []

    for vehicle in ranked_vehicles:
        """get OD-pair(s,t) of the vehicle"""
        current_edge = traci.vehicle.getRoadID(vehicle)
        original_route = list(traci.vehicle.getRoute(vehicle))
        target_edge = original_route[-1]
        # get k_shortest_paths
        if current_edge == target_edge:
            continue
        K_paths = paths[(current_edge,target_edge)]
        if len(K_paths) == 1:
            continue
        
        
        """select the optimal route from k paths"""        
        newPath,predicted_speeds = pp.pickPath(vehicle, K_paths, speed_result, RS_info, TL_info, conn_TL,\
         Coordinate, RSDensities, vehicleRS_dict[vehicle]["RS_from"],vehicleRS_dict[vehicle]["RS_to"],predicted_speeds)
        #return diff ,same
        getindex = original_route.index(current_edge)
        orig_path = original_route[getindex:]
        if newPath == orig_path :
            same += 1
        #    print("---------------------select the same-------------------")
        else:
            diff += 1
    
    gc.collect()

    return diff ,same
"""
footprint f(RS) = n(RS) * w(RS)
w(RS) = avg_RS_len/len(RS) * lane_num(RS)

footprint_dict = {RS_id:[n(RS), w(RS)]}    
"""  
def initializeFootprint(RS_info):
    
    footprint_dict = {}
    avg_RS_len = 121.49643493761141    
    '''
    """compute avg_RS_len"""
    for RS in RS_info.values():
        RS_len.append(float(RS[2]))
    avg_RS_len = np.mean(RS_len)  
    #print("avg_RS_len: ",avg_RS_len)
    '''
    for RS_id, info_list in RS_info.items():
        footprint_dict.update({RS_id:[0, avg_RS_len/float(info_list[2])*float(info_list[3])]})        
    #print("initializeFootprint: ",footprint_dict)
    return footprint_dict
       

def updateFootprint(newPath, footprint_dict):
    
    for RS in newPath:
        fplist = footprint_dict[RS]
        footprint_dict[RS] = [fplist[0]+1, fplist[1]]
    
    return footprint_dict

def SetGraphWeight(RN,RSDensities,con_info,Zone_dict):
    occ_list = []
    EdgetoZoneOcc_dict = {}
    """ count Zone occupancy """
    for zone , edge_list in Zone_dict.items():
        vehicleNum = 0
        N_max = 0
        for RS in edge_list:
            N_max += float(RSDensities[RS]["Length"])*float(RSDensities[RS]["Lane_Num"])/(5.0+2.5)
            vehicleNum += float(RSDensities[RS]["Veh_Num"])
        ZoneOcc = vehicleNum / N_max
        if ZoneOcc > 1:
            ZoneOcc = 1
        for RS in edge_list:
            EdgetoZoneOcc_dict[RS] = ZoneOcc
    
    for conn in con_info:
        
        """"weight = 0 , occupancy = 0,  length = Normalization of length , len_CV """
        """ count road occupancy """        
        RS = conn[0]
        N_max = float(RSDensities[RS]["Length"])*float(RSDensities[RS]["Lane_Num"])/(5.0+2.5)
        ro = float(RSDensities[RS]["Veh_Num"])/N_max
        if ro > 1:
            ro = 1
        #Normalize_occ = (ro - Min_occ ) / (Max_occ - Min_occ)
        weight_occ = 1/3 
        weight_len = 1/3
        weignt_zone = 1/3
        
        RN[conn[0]][conn[1]]["weight"] = RN[conn[0]][conn[1]]["length"] * weight_len +  ro * weight_occ + EdgetoZoneOcc_dict[RS] * weignt_zone #Normalize_occ *  weight_occ
        
    return RN


"""ODpairs:[(O,D)]"""
def getODpairs(ranked_vehicles):
    
    ODpairs = []
    """get OD-pair(s,t) of the vehicle"""
    for veh in ranked_vehicles:
        
        current_edge = traci.vehicle.getRoadID(veh)
        original_route = list(traci.vehicle.getRoute(veh))
        target_edge = original_route[-1]
        if current_edge == target_edge:
            continue
        ODpairs.append((current_edge,target_edge))
    
    #delete duplicated element
    ODpairs = sorted(set(ODpairs), key = ODpairs.index)
    
    return ODpairs

"""paths:{(O,D):[[path]*K]}"""    
def getAllKSP(RN, ODpairs, K):
    
    paths = {}
    
    for OD in ODpairs:
        if OD not in paths.keys():
                paths[OD] = k_shortest_paths(RN, OD[0], OD[1], K, "weight")
    return paths

def getAllRS(paths):
    
    RS_list = []
    for K_paths in paths.values():
        for path in K_paths:
            for edge in path:
                if edge not in RS_list:
                    RS_list.append(edge)
    
    return RS_list

"""KSP"""
def k_shortest_paths(G, source, target, k, weight=None):
    
    return list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))











    