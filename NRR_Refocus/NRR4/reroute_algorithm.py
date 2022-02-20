# -*- coding: utf-8 -*-
import traci
import numpy as np
import networkx as nx
from itertools import islice

import gc
import pick_path as pp
from tensorflow.python.keras.models import load_model

"""do Reroute"""
def Reroute(RS_info, TL_info, conn_TL, paths, speed_result, ranked_vehicles, Coordinate, RSDensities, vehicleRS_dict, diff, same, RS_list,NextRoad_dict):
    
    for vehicle in ranked_vehicles:
        current_edge = traci.vehicle.getRoadID(vehicle)
        original_route = list(traci.vehicle.getRoute(vehicle))
        target_edge = original_route[-1]
        # get k_shortest_paths
        if current_edge == target_edge:
            continue
        NextRS_list = NextRoad_dict[current_edge]
        if len(NextRS_list) == 1:
            continue
        GDpath_list = []
        for RS in NextRS_list:
            try:
                GDpath_list.append(paths[(RS,target_edge)])
            except:
                pass
        if len(GDpath_list) == 1:
            continue

        newPath = pp.pickPath(vehicle, GDpath_list, speed_result, RS_info, TL_info, conn_TL,\
         Coordinate, RSDensities, vehicleRS_dict[vehicle]["RS_from"],vehicleRS_dict[vehicle]["RS_to"],NextRoad_dict)

        getindex = original_route.index(current_edge)
        orig_path = original_route[getindex:]
        '''
        print("vehicle orign-------------------")
        print(orig_path)
        print("-----------------------")
        '''
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



"""ODpairs:[(O,D)]"""
def getnextroad(ranked_vehicles,RN):
    adjacency_dictionary = {} #get the adjacency dictionary of the graph
    NextRoad_dict = {}
    for n, nbrdict in RN.adjacency():
        """make the adjacency_dictionary like {node:[node_nbr1,node_nbr2,...]}"""
        adjacency_dictionary.update({n : list(nbrdict.keys())})
    """
    NextRoad_dict
    current_edge: next_edge1 , next_edge2 ,.... 
    """
    for veh in ranked_vehicles:
        
        current_edge = traci.vehicle.getRoadID(veh)
        road_list = adjacency_dictionary[current_edge]
        if current_edge not in NextRoad_dict:
            NextRoad_dict.update({current_edge:road_list})
    
    return NextRoad_dict
        

def getODpairs(ranked_vehicles ,NextRoad_dict):
    
    ODpairs = []
    """get OD-pair(s,t) of the vehicle"""
    for veh in ranked_vehicles:
        
        current_edge = traci.vehicle.getRoadID(veh)
        
        original_route = list(traci.vehicle.getRoute(veh))
        target_edge = original_route[-1]
        if current_edge == target_edge:
            continue
        NextRS_list = NextRoad_dict[current_edge]
        for RS in NextRS_list:
            ODpairs.append((RS,target_edge))
    
    #delete duplicated element
    ODpairs = sorted(set(ODpairs), key = ODpairs.index)
    
    return ODpairs
"""paths:{(O,D):[path]}"""    
def getSP(RN, ODpairs):
    paths = {}
    
    for OD in ODpairs:
        if OD not in paths.keys():
            try:
                paths[OD] = nx.shortest_path(RN, source=OD[0], target= OD[1],weight = "weight")
            except:
                pass

    return paths

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

# ??for-loop 裡�?�?load model ??del model 來�?�?predicted_speeds
def getAllRSspeeds(RS_list, speed_result):
    
    predicted_speeds = {}
    
    for RS in RS_list:
        model = load_model('data/model.h5')
        past_speed_list = speed_result[RS]
        X = [[[13.89]*1 for i in range(12)]]
        #如�??�去資�?不�?，於?�方補�?13.89
        if (len(past_speed_list))<12:
            new_idx = 12-len(past_speed_list)
            for idx,i in enumerate(past_speed_list):
                X[0][new_idx][0] = i
                new_idx +=1
        else:
            for idx,i in enumerate(past_speed_list[-12:]):
                X[0][idx][0] = i
        prediction = model.predict(np.array(X))

        predicted_speeds[RS] = prediction
        
        del model
    gc.collect
    return predicted_speeds










    