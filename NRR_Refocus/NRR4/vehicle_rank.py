# -*- coding: utf-8 -*-
"""
Created on Thu Jun  4 19:23:26 2020

@author: user
"""
import traci
import random
import math
""" if vehicle is choosed exceed once then we choose the congested road neared by vehicle"""
def ChooseNearCongRoad(selected_vehicles, vehicleRS_dict, RS_info, Coordinate):
    for vehicle in selected_vehicles:

        current_edge = traci.vehicle.getRoadID(vehicle)
        CurrentEdge_EndNode = RS_info[current_edge][1]
        CongEdge_StartNode_list = vehicleRS_dict[vehicle]["RS_from"]
        if len(CongEdge_StartNode_list) == 1:
            vehicleRS_dict[vehicle]["RS_from"] = vehicleRS_dict[vehicle]["RS_from"][0]
            vehicleRS_dict[vehicle]["RS_to"] = vehicleRS_dict[vehicle]["RS_to"][0]
        else:
            mindistance = 999999
            minidx = -1 
            CurrentEdge_EndXY = Coordinate[CurrentEdge_EndNode]
            for idx , CongEdge_StartNode in enumerate(CongEdge_StartNode_list):
                CongEdge_StartXY = Coordinate[CongEdge_StartNode]

                distance = math.sqrt(math.pow(CurrentEdge_EndXY[0]-CongEdge_StartXY[0],2) + math.pow(CurrentEdge_EndXY[1]-CongEdge_StartXY[1],2))
                if distance < mindistance:
                     mindistance = distance 
                     minidx = idx
            vehicleRS_dict[vehicle]["RS_from"] = vehicleRS_dict[vehicle]["RS_from"][minidx]
            vehicleRS_dict[vehicle]["RS_to"] = vehicleRS_dict[vehicle]["RS_to"][minidx]

    return vehicleRS_dict

def rankVehicles(selected_vehicles, RS_info):
    
    ranked_vehicles = []
    svdict = {}
    selected_svdict = {}
    
    for vehicle in selected_vehicles:
        
        remain_distance = 0
        
        vehicle_route = list(traci.vehicle.getRoute(vehicle))
        RS_index = vehicle_route.index(traci.vehicle.getRoadID(vehicle))
        remain_route = vehicle_route[RS_index+1:]
        for RS in remain_route:
            remain_distance = remain_distance + float(RS_info[RS][2])
            
        svdict.update({vehicle:remain_distance})
   
    selected_svdict = {k: v for k, v in sorted(svdict.items(), key=lambda item: item[1],reverse=True)}
    #print(selected_svdict)
    ranked_vehicles = list(selected_svdict.keys())
    
    return ranked_vehicles


def rankVehiclessd(selected_vehicles, RS_info):
    
    ranked_vehicles = []
    svdict = {}
    selected_svdict = {}
    
    for vehicle in selected_vehicles:
        
        remain_distance = 0
        
        vehicle_route = list(traci.vehicle.getRoute(vehicle))
        RS_index = vehicle_route.index(traci.vehicle.getRoadID(vehicle))
        remain_route = vehicle_route[RS_index+1:]
        for RS in remain_route:
            remain_distance = remain_distance + float(RS_info[RS][2])
            
        svdict.update({vehicle:remain_distance})
   
    selected_svdict = {k: v for k, v in sorted(svdict.items(), key=lambda item: item[1])}
    #print(selected_svdict)
    ranked_vehicles = list(selected_svdict.keys())
    
    return ranked_vehicles
