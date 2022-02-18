# -*- coding: utf-8 -*-
"""
Created on Thu Jun  4 15:22:19 2020

@author: user
"""
import networkx as nx
import traci
def checkdirection(RS_info,Coordinate,current_edge,choose_edge):
     CurrentEdge_StartNode = RS_info[current_edge][0]
     CurrentEdge_EndNode = RS_info[current_edge][1]
     
     ChooseEdge_StartNode = RS_info[choose_edge][0]
     ChooseEdge_EndNode = RS_info[choose_edge][1]
     
     #list
     CurrentEdge_StartXY = Coordinate[CurrentEdge_StartNode]
     CurrentEdge_EndXY = Coordinate[CurrentEdge_EndNode]
     
     ChooseEdge_StartXY = Coordinate[ChooseEdge_StartNode]
     ChooseEdge_EndXY = Coordinate[ChooseEdge_EndNode]
     
     # vector format is in first quadrant
     if CurrentEdge_EndXY[1] >= CurrentEdge_StartXY[1] and CurrentEdge_EndXY[0] >= CurrentEdge_StartXY[0] :
         # slope > 1
         if (CurrentEdge_EndXY[1] - CurrentEdge_StartXY[1]) / (CurrentEdge_EndXY[0] - CurrentEdge_StartXY[0]) > 1:
             if ChooseEdge_EndXY[1] <= CurrentEdge_StartXY[1]:
                 return True
             else:
                 return False
         else:
            if ChooseEdge_EndXY[0] <= CurrentEdge_StartXY[0]:
                 return True
            else:
                 return False
     # vector format is in second quadrant
     elif CurrentEdge_EndXY[1] >= CurrentEdge_StartXY[1] and CurrentEdge_EndXY[0] < CurrentEdge_StartXY[0] :
         # slope < -1
         if (CurrentEdge_EndXY[1] - CurrentEdge_StartXY[1]) / (CurrentEdge_EndXY[0] - CurrentEdge_StartXY[0]) < -1:
             if ChooseEdge_EndXY[1] <= CurrentEdge_StartXY[1]:
                 return True
             else:
                 return False
         else:
            if ChooseEdge_EndXY[0] >= CurrentEdge_StartXY[0]:
                 return True
            else:
                 return False
     # vector format is in third quadrant
     elif CurrentEdge_EndXY[1] < CurrentEdge_StartXY[1] and CurrentEdge_EndXY[0] < CurrentEdge_StartXY[0] :
         # slope > 1
         if (CurrentEdge_EndXY[1] - CurrentEdge_StartXY[1]) / (CurrentEdge_EndXY[0] - CurrentEdge_StartXY[0]) > 1 :
             if ChooseEdge_EndXY[1] >= CurrentEdge_StartXY[1]:
                 return True
             else:
                 return False
         else:
            if ChooseEdge_EndXY[0] >= CurrentEdge_StartXY[0]:
                 return True
            else:
                 return False
     # vector format is in fourth quadrant
     elif CurrentEdge_EndXY[1] < CurrentEdge_StartXY[1] and CurrentEdge_EndXY[0] >= CurrentEdge_StartXY[0] :
         # slope < -1
         if (CurrentEdge_EndXY[1] - CurrentEdge_StartXY[1]) / (CurrentEdge_EndXY[0] - CurrentEdge_StartXY[0]) < -1:
             if ChooseEdge_EndXY[1] >= CurrentEdge_StartXY[1]:
                 return True
             else:
                 return False
         else:
            if ChooseEdge_EndXY[0] <= CurrentEdge_StartXY[0]:
                 return True
            else:
                 return False
     print("there is a problem")
     exit(0)
     return False

"""get affected RS"""
def getAffectedRSviaedge(current_edge, RN, upstream_level, RS_info, Coordinate):
    
    adjacency_dictionary = {} #get the adjacency dictionary of the graph
    level_dictionary = {} # {level:[affected_RS1, affected_RS2, ...]}
    
    RN_rev = nx.DiGraph.reverse(RN)
    
    for n, nbrdict in RN_rev.adjacency():
        """make the adjacency_dictionary like {node:[node_nbr1,node_nbr2,...]}"""
        adjacency_dictionary.update({n : list(nbrdict.keys())})
    
    # do level1
    level1 = adjacency_dictionary[current_edge]
    level_dictionary.update({1:level1})
    # current level
    i = 2 
    
    # do the rest 
    while i <= upstream_level:
        
        edgelist = []
        leveli = []
        
        for current in level1:
            for edge in adjacency_dictionary[current]:
                if checkdirection(RS_info,Coordinate,current_edge,edge):
                    edgelist.append(edge)
        #delete duplicated element
        leveli = sorted(set(edgelist), key = edgelist.index)
        level1 = leveli
        level_dictionary.update({i:leveli})
        i +=1
        
    return level_dictionary

"""select affected vehicles"""    
def selectVehicles(conRS, level_dictionary, vehicleRS_dict, RS_from, RS_to):
    selected_vehicles = []
    allRSlist = []
    
    
    for RSlist in level_dictionary.values():
        allRSlist = allRSlist + RSlist
    
    #delete duplicated element
    allRSlist = sorted(set(allRSlist), key = allRSlist.index)
        
    vehicles = []
    
    
    for RS in allRSlist:
        vehicles = vehicles + list(traci.edge.getLastStepVehicleIDs(RS))

    vehicles = list(set(vehicles))
    for veh in vehicles:
        selected_vehicles.append(veh)
        if veh not in vehicleRS_dict:
            vehicleRS_dict[veh] = {}
            vehicleRS_dict[veh]["RS_from"] = []
            vehicleRS_dict[veh]["RS_to"] = []
        
        vehicleRS_dict[veh]["RS_from"].append(RS_from)
        vehicleRS_dict[veh]["RS_to"].append(RS_to)

    '''
    for veh in vehicles:
        route = traci.vehicle.getRoute(veh)
        if conRS in route:
            idx = route.index(conRS)
            currRS = traci.vehicle.getRoadID(veh)
            if currRS in route[:idx]:
                selected_vehicles.append(veh)
        
                if veh not in vehicleRS_dict:
                    vehicleRS_dict[veh] = {}
                    vehicleRS_dict[veh]["RS_from"] = []
                    vehicleRS_dict[veh]["RS_to"] = []
        
                vehicleRS_dict[veh]["RS_from"].append(RS_from)
                vehicleRS_dict[veh]["RS_to"].append(RS_to)
    '''
                    
    return selected_vehicles ,vehicleRS_dict
    









    
    
    
    
    
    