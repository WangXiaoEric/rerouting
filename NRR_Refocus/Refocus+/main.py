from __future__ import division
import os
import ast
import sys
import subprocess
import signal
import socket
import logging
import _thread as thread
import time
import tempfile
import math
import random
import networkx as nx
from collections import defaultdict, deque
from math import log
from itertools import islice
#from k_shortest_paths import k_shortest_paths
from optparse import OptionParser
from bs4 import BeautifulSoup
import lxml
from collections import defaultdict
from decimal import Decimal
index = 0
avgLengthall = 0
avgSpeedall = 0
dict_edgeRSUs = {}
list_predict = []
dict_lane ={}
listfN=[]
laneallgraph =[]
edgeallgraph =[]
edgeallsgraph =[]
visit_bfs =[]
dict_fc ={}
list_source ={}
list_present_network =[]
list_vehicle_set_route =[]
number_of_lane ={}
TMax=0
footprintList =[]
dict_fc ={}
edgListLength =[]
edgListspeed = []
listcdm=[]
listtraveltime=[]
listNtraveltime=[]
tonodedict={}
listEdgeRSU = []
dict_road_conges_traffic_area={}
# We need to import Python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Environment variable SUMO_HOME not defined")
    
import traci
import sumolib
from sumolib import checkBinary 
def initializeData():
    global avgLengthall
    global avgSpeedall
    global dict_edgeRSUs
    global list_predict
    global dict_lane
    global listfN
    global laneallgraph
    global edgeallgraph
    global edgeallsgraph
    global visit_bfs
    global dict_fc
    global list_source
    global list_present_network
    global list_vehicle_set_route
    global number_of_lane
    global TMax
    global footprintList
    global edgListLength
    global edgListspeed
    global listcdm
    global listtraveltime
    global listNtraveltime
    global tonodedict
    global listEdgeRSU
    global dict_road_conges_traffic_area


    avgLengthall = 0
    avgSpeedall = 0
    dict_edgeRSUs = {}
    list_predict = []
    dict_lane ={}
    listfN=[]
    laneallgraph =[]
    edgeallgraph =[]
    edgeallsgraph =[]
    visit_bfs =[]
    dict_fc ={}
    list_source ={}
    list_present_network =[]
    list_vehicle_set_route =[]
    number_of_lane ={}
    TMax=0
    footprintList =[]
    edgListLength =[]
    edgListspeed = []
    listcdm=[]
    listtraveltime=[]
    listNtraveltime=[]
    tonodedict={}
    listEdgeRSU = []
    dict_road_conges_traffic_area={}

def k_shortest_paths(G, source, target, k, weight=None):
    
    return list(islice(nx.shortest_simple_paths(G, source, target, weight=weight), k))

def bfs_edges(G, source, reverse=False, L=0):#BFSMAXDEPTH program
    #print("enter bfs")
    global visit_bfs
    visit_bfs = []
    
    if reverse : 
        neighbors = G.predecessors
    else:
        neighbors = G.successors
    visited = set()
    visited.add(source)
    
    
    #del visit_bfs[:]
    
    visit_bfs.append(source)
    
    #print("123123")
    queue = deque([(source, neighbors(source), 0)])
    
    while queue: 
        
        parent, children, v = queue[0]
        q= children
        front= queue.popleft()
        level = front[2]
        
        if level >= L:
              break
        level +=1
        
        for child in children:  
            if child not in visited:
                visited.add(child)
                visit_bfs.append(child)
                queue.append((child, neighbors(child), level))
    #'''
    

def choose_route(paths_and_weights, possible_paths, k, max_weight,begin_of_cycle,graph,source, destination,net, temperature=1):  
    route = None
    global dict_fc
    global avgLengthall
    global avgSpeedall
    global edgeallgraph
    highest_choice_prob = 0
    N=0
    list_choice_prob=[]
    dict_choice_prob={}
    ##sys.stdout = open('h.txt','wt')
    for edge in edgeallgraph:
         N += dict_fc[edge]         
    if N==0:
       route = nx.dijkstra_path(graph, source, destination,"weight")
       for e in route:
         dict_fc[e] = dict_fc[e] + 1
         #logging.debug("dict_fc::::(%d)" % (dict_fc[e]))
    else:
      for path in possible_paths:
        divefN= 0
        for edge in path:
          ni = dict_fc[edge]
          fci=0.0
          #print("ni:{0}" .format(ni))
          Lenght_Lane = traci.lane.getLength(edge + '_' + '0')
          max_speed_Lane = float(traci.lane.getMaxSpeed(edge + '_' + '0'))
          edge_road = net.getEdge(edge)
          N_Lane = edge_road.getLaneNumber()
          fci = Decimal(ni * (Decimal(Decimal(avgLengthall) / Decimal(Decimal(Lenght_Lane) * Decimal(N_Lane))) * Decimal(Decimal(avgSpeedall) / Decimal(max_speed_Lane))))
          #print("fci:{0}" .format(fci))
          #logging.debug("fci::::(%d)" % (fci))
          #logging.debug("N::::(%d)" % (N))
          if fci==0 : continue
          divefN += Decimal(Decimal(Decimal(fci) / N) * Decimal(math.log(Decimal(fci) / N)))
          #print("divefN:{0}" .format(divefN))

        divefN = -Decimal(divefN)
        #print(divefN)
        Entropyk = Decimal(divefN)
        #print("Entropyk:{0}" .format(Entropyk))
        choice_prob = Decimal(math.exp(Entropyk))
        #print("choice_prob:{0}" .format(choice_prob))
        list_choice_prob.append(choice_prob)
        dict_choice_prob[choice_prob]= path
      #print("route:{0}" .format(route))
      min_choice_prob = min(list_choice_prob)
      route = dict_choice_prob[min_choice_prob]
      for edge in route:
         dict_fc[edge]+=1
    return route    
    
            

   
    
 

def get_paths_and_weights(graph, simple_paths):
    paths_and_weights = {}
    for path in simple_paths:
        weight = 0
        for i in range(len(path) - 1):
            u = path[i]
            v = path[i + 1]
            weight += graph[u][v]["TT"]
        paths_and_weights[tuple(path)] = weight
    return paths_and_weights 
 


def get_path_length(G, path, weight='weight'):
    length = 0
    if len(path) > 1:
        for i in range(len(path) - 1):
            u = path[i]
            v = path[i + 1]
            
            length += G.edge[u][v].get(weight, 1)
    
    return length    
    
def cloud_server(vehicle,route, source, destination,congestedRoadsss, graph, begin_of_cycle, net):
 aux=[]
 Route=route[route.index(source):route.index(destination)]
 global list_vehicle_set_route
 ##logging.debug("RRRoute in cloud request rsu ::::(%s)" % (Route))
 for road in Route: 
  if len(aux) > 0:continue
  if road in congestedRoadsss: 
    if source != destination:
        paths=[]
        simple_paths=[]
        ##sys.stdout = sys.__stdout__
        try:
            k_paths = k_shortest_paths(graph, source, destination, 4, "TT")
        except:
          continue
        for path in k_paths:
           paths.append(path)
           simple_paths.append(path)
        paths_and_weights = get_paths_and_weights(graph, simple_paths)
        weights = paths_and_weights.values()
        max_weight = max(weights)
        k = 0#calculate_boltzmann_k_value(weights, max_weight)

        #logging.debug("Calculating choose_route for pair cloud_server that request RSU(%s, %s, %s)" % (source, destination, vehicle))
        new_route = choose_route(paths_and_weights, simple_paths, k, max_weight, begin_of_cycle, graph, source, destination, net)
        #aux += route[0:route.index(source)]
        aux += new_route
        #logging.debug("path for pair cloud_server that request RSU(%s, %s, %s, %s)" % (source, destination, vehicle, aux))
        traci.vehicle.setRoute(vehicle, aux)
        list_vehicle_set_route.append(vehicle)
        
    
             
  
def reroute_vehicles_RSUs(subgraph_g,list_of_vehicle,edgelist,congestedRoadsss, congestedRoads,graph,buffered_paths, begin_of_cycle, net):######################
 simple_paths = []
 listvehicle=[]
 listcongesvehicle=[]
 global list_vehicle_set_route
 #sys.stdout = open('edgee.txt','wt')
 for vehicle in list_of_vehicle:
  aux=[] 
  route=[]
  del aux[:]
  del route[:]
  source = traci.vehicle.getRoadID(vehicle)
  if source.startswith(":"): continue
  route = traci.vehicle.getRoute(vehicle)
  ##logging.debug("Route::::(%s)" % (route))
  destination = route[-1]
  ##logging.debug("edgelist::::(%s)" % (edgelist))
  L=[]
  Route=route[route.index(source):route.index(destination)]
  ##logging.debug("RRRoute::::(%s)" % (Route))
  if destination in edgelist:
    #print("a")
    for road in Route:  
     if len(aux) > 0:continue
     if road in congestedRoads:#route:
       if source != destination:
        #logging.debug("Calculating shortest paths for pair (%s, %s)" % (source, destination))
        paths=[]
        simple_paths=[]
        try:
            k_paths = k_shortest_paths(subgraph_g, source, destination, 4, "TT")
        except:
            continue
        if len(k_paths) == 1:
         ##logging.debug("no reroute for pair in RSU (%s, %s,%s)" % (source, destination, vehicle))
         #auxr= route[0:route.index(destination)]
         #traci.vehicle.setRoute(vehicle,auxr)
         ##logging.debug("set path no reroute pair in RSU (%s, %s,%s ,%s)" % (source, destination, vehicle,auxr))
         continue
        else:
          for path in k_paths:
           paths.append(path)
           simple_paths.append(path)
          paths_and_weights = get_paths_and_weights(subgraph_g, simple_paths)
          weights = paths_and_weights.values()
          max_weight = max(weights)
          k = 0#calculate_boltzmann_k_value(weights, max_weight)
          #logging.debug("Calculating choose_route for pair in RSU::::: (%s, %s, %s)" % (source, destination, vehicle))
          new_route = choose_route(paths_and_weights, simple_paths, k, max_weight, begin_of_cycle, subgraph_g, source, destination, net)
 
          #logging.debug("set route for pair in RSU:::: (%s, %s,%s)" % (source,destination, vehicle))
          #aux = route[0:route.index(source)]
          aux += new_route
          #logging.debug(" aux for pair in RSU ::::::%s" % aux)
          traci.vehicle.setRoute(vehicle, aux)
          list_vehicle_set_route.append(vehicle)
  else:
    #print("b")  
    cloud_server(vehicle,route, source, destination,congestedRoadsss, graph, begin_of_cycle, net)
    
  
           
    
def cloud_reroute_vehicles(graph,list_of_vehiclecloud,congestedRoadsss, buffered_paths, begin_of_cycle, net):###########################
    simple_paths = []
    listvehicle=[]
    listcongesvehicle=[]
    global list_vehicle_set_route
    for vv in list_of_vehiclecloud:
     for v in vv:
      listvehicle.append(v)
   
    
    for vehicle in listvehicle:
        aux=[]
        source = traci.vehicle.getRoadID(vehicle)
        if source.startswith(":"): continue
        route = traci.vehicle.getRoute(vehicle)
        destination = route[-1]
        Route=route[route.index(source):route.index(destination)]
        #logging.debug("RRRoute in cloud ::::(%s)" % (Route))
        for road in route: 
         if len(aux) > 0:continue
         if road in congestedRoadsss: 
          if source != destination:
            #logging.debug("Calculating shortest paths for pair in cloud server (%s, %s ,%s)" % (source, destination, vehicle))
            paths = []
            simple_paths = []
            try:
                k_paths = k_shortest_paths(graph, source, destination, 4, "TT")
            except:
              continue
            for path in k_paths:
             #print(path)
             paths.append(path)
             simple_paths.append(path)
            paths_and_weights = get_paths_and_weights(graph, simple_paths)
            weights = paths_and_weights.values()
            max_weight = max(weights)
            k = 0#calculate_boltzmann_k_value(weights, max_weight)
            new_route = choose_route(paths_and_weights, simple_paths, k, max_weight, begin_of_cycle, graph,source, destination, net)
            #print(new_route)
            #print("-------------------------")
            #new_route = nx.dijkstra_path(graph, source, destination,"weight")
            #aux = route[0:route.index(source)]
            aux += new_route
            traci.vehicle.setRoute(vehicle, aux)
            list_vehicle_set_route.append(vehicle)
            
             
        
            
             
        
def Update_Travel_Time(graph, net): # Road Weight Measurement
 global edgListLength
 maxLenght = max(edgListLength)
 global listtraveltime
 global listEdgeRSU
 global dict_road_conges_traffic_area
 global number_of_lane
 for road in graph.nodes():
  for successor_road in graph.successors(road):
   minold= min(listtraveltime)
   maxold= max(listtraveltime)
   minnew = minold
   rescale= ((1-minnew)/ (maxold - minold))* (graph[road][successor_road]["weight"] - minold) + minnew  
   graph[road][successor_road]["weight"] = rescale
   if road in listEdgeRSU:
    Lenght_Lane = traci.lane.getLength(road + '_' + '0')
    N_Lane = number_of_lane[road]
    Kjam = float ((Lenght_Lane * N_Lane)/ (7.5))
    number_vehicles = traci.edge.getLastStepVehicleNumber(road)
    if number_vehicles !=0 and Lenght_Lane >= 9:
     Density = float (number_vehicles / Kjam)
    else:
     Density = 0.0
    if road.startswith(":") or successor_road.startswith(":"):
     if dict_road_conges_traffic_area[road] >= 0.3:
      graph[road][successor_road]["TT"] = (math.exp(dict_road_conges_traffic_area[road])) + ((graph[road][successor_road]["weight"])* 2 )
     else:
      graph[road][successor_road]["TT"] = (math.exp(graph[road][successor_road]["weight"])) + ((dict_road_conges_traffic_area[road])* 2 )
    else:
     if dict_road_conges_traffic_area[road] >= 0.3:
      graph[road][successor_road]["TT"] = (math.exp(dict_road_conges_traffic_area[road])) + ((graph[road][successor_road]["weight"]) * 2 )  + ((graph[road][successor_road]["predict"]+graph[road][successor_road]["predictArea"] )* 2)  
     else:
      graph[road][successor_road]["TT"] = (math.exp(graph[road][successor_road]["weight"] ))  + ((dict_road_conges_traffic_area[road] )* 2 ) + ((graph[road][successor_road]["predict"]+graph[road][successor_road]["predictArea"] )* 2) 
   else:
    Lenght_Lane = traci.lane.getLength(road + '_' + '0')
    Max_speed_LaneZ = traci.lane.getMaxSpeed(road + '_' + '0')
    MeanSpeedZ= traci.edge.getLastStepMeanSpeed(road)
    if road.startswith(":"):
     N_Lane = number_of_lane[road]
    else:
     edge_road = net.getEdge(road)
     N_Lane = edge_road.getLaneNumber()
    #logging.debug("N_Lane[edge]::::(%s)" % (N_Lane))
    Kjam = float ((Lenght_Lane * N_Lane)/ (7.5))
    number_vehicles = traci.edge.getLastStepVehicleNumber(road)
    Wedgev = 1 - (MeanSpeedZ / Max_speed_LaneZ)
    if number_vehicles !=0 and Lenght_Lane >= 9:
     Density = float (number_vehicles / Kjam)
    else:
     Density = 0.0
    if Wedgev <0:
      Wedgev = 0
    if Wedgev >1:
      Wedgev = 1
    if Density <0:
      Density = 0
    if Density > 1:
      Density = 1

    sum_conges= (Wedgev+Density)/2
    if road.startswith(":") or successor_road.startswith(":"):
             if sum_conges >= 0.3:
              graph[road][successor_road]["TT"] = (math.exp(sum_conges)) + (graph[road][successor_road]["weight"]*2) 
             else:
              graph[road][successor_road]["TT"] = (math.exp(graph[road][successor_road]["weight"] ) ) + (sum_conges  * 2)
    else:
     if sum_conges >= 0.3:
      graph[road][successor_road]["TT"] = (math.exp(sum_conges) ) + ((graph[road][successor_road]["weight"] + graph[road][successor_road]["predict"] +graph[road][successor_road]["predictArea"])* 2)   
             
     else:
      graph[road][successor_road]["TT"] = (math.exp(graph[road][successor_road]["weight"]  ) ) +( (sum_conges * 2)) + ((graph[road][successor_road]["predict"] +graph[road][successor_road]["predictArea"])* 2)  
               
   d = graph[road][successor_road]["TT"]
    
def update_travel_time_on_roads(graph, time, begin_of_cycle):
    for road in graph.nodes():
        travel_time = traci.edge.getAdaptedTraveltime(road, time)
        if travel_time <= 0:
            travel_time = traci.edge.getTraveltime(road)
        
        for successor_road in graph.successors(road):
            if begin_of_cycle:
                graph[road][successor_road]["TTg"] = travel_time
            else:
                t = (graph[road][successor_road]["TTg"] + travel_time) / 2
                t = t if t > 0 else travel_time
                graph[road][successor_road]["TTg"] = t

def road_congestion_at_traffic_area(road_graph_travel_time, edgelist, zone_conges, RSU_ids,net):     
  global dict_road_conges_traffic_area    
  global listEdgeRSU           
  for rsuid in RSU_ids:
    for edge in edgelist[rsuid]:
       listEdgeRSU.append(edge)
       Lenght_Lane = traci.lane.getLength(edge + '_' + '0')
       Max_speed_LaneZ = traci.lane.getMaxSpeed(edge + '_' + '0')
       MeanSpeedZ= traci.edge.getLastStepMeanSpeed(edge)
       edge_road = net.getEdge(edge)
       N_Lane = edge_road.getLaneNumber()
       #logging.debug("N_Lane[edge]::::(%s)" % (N_Lane))
       Kjam = float ((Lenght_Lane * N_Lane)/ (7.5))
       number_vehicles = traci.edge.getLastStepVehicleNumber(edge)
       Wedgev = 1 - (MeanSpeedZ / Max_speed_LaneZ)
       if number_vehicles !=0 and Lenght_Lane >= 9:
         Density = float (number_vehicles / Kjam)
       else:
         Density = 0.0

       if Wedgev <0:
          Wedgev = 0
       if Wedgev >1:
          Wedgev = 1
       if Density <0:
          Density = 0
       if Density > 1:
          Density = 1 

       ZoneC = zone_conges[rsuid]
       sum_conges= (Wedgev+ZoneC+Density)/3
       dict_road_conges_traffic_area[edge] = sum_conges
       

def prediction_on_edge(graph, net):
 # Input  
 global index
 global visit_bfs
 global list_predict
 
 f = open('data/01/od_route_file.odtrips'+str(index)+'.rou.xml', 'r')
 data = f.read()
 soup = BeautifulSoup(data,'lxml')
 f.close()
 dict_edge_deparr={}
 #interval_tag= soup.findAll("vehicle")
 for vehicle_tag in soup.findAll("vehicle"):
      vehicle_id = vehicle_tag["id"]
      vehicle_route = vehicle_tag.find("route")
      vehicle_edges = vehicle_route["edges"]
      dict_edge_deparr[vehicle_id]= vehicle_edges
 
 # # f = open('joined.rou.xml', 'r')
 # # data = f.read()
 # # soup = BeautifulSoup(data)
 # # f.close()
 # # #dict_edge_deparr={}
 # # #interval_tag= soup.findAll("vehicle")
 # # for vehicle_tag in soup.findAll("vehicle"):
      # # vehicle_id = vehicle_tag["id"]
      # # vehicle_route = vehicle_tag.find("route")
      # # vehicle_edges = vehicle_route["edges"]
      # # dict_edge_deparr[vehicle_id]= vehicle_edges # insert route of vehicle to dictionary
      ##logging.debug("interval_end::::(%s)" % (interval_end))
      # #logging.debug("endp::::(%s)" % (endp))
      # #logging.debug("beginp::::(%s)" % (beginp))
      # #logging.debug("interval_begin::::(%s)" % (interval_begin))
      #if (interval_begin == beginp and interval_end == endp):
      # edge_tag = interval_tag.findAll("edge")
      # for edge in edge_tag:
       # edge_id = edge["id"]
       # ##logging.debug("edge_idedge_id::::(%s)" % (edge_id))

       # edge_departed = edge["departed"]
       # edge_arrived = edge["arrived"]
       # dict_edge_deparr[edge] = (edge_departed, edge_arrived)
       ##logging.debug("dict_edge_deparr[edge]::::(%s)" % (dict_edge_deparr[edge]))

       
          
           # edge_entered = edge["entered"]
           # edge_left = edge["left"]
           # dict_edge_enterleft[edge] = (edge_entered, edge_left)
          
 departed_ids = traci.simulation.getDepartedIDList() 
 arrived_ids = traci.simulation.getArrivedIDList() 
 for road in graph.nodes():
  if road.startswith(":"):
   for successor_road in graph.successors(road):
     graph[road][successor_road]["predict"]=0
  else:
     edge_road = net.getEdge(road)
     N_Lane = edge_road.getLaneNumber()
     #logging.debug("N_Lane[edge]::::(%s)" % (N_Lane))
     C_i=0
     X_arrived_i = 0
     X_departed_i = 0
     
     for depart in departed_ids:
      #logging.debug("depart::::(%s)" % (depart))
      route = traci.vehicle.getRoute(depart)
      initialpos= route[0]
      if initialpos == road :
        X_departed_i +=1
      else:
        X_departed_i += 0
      
      
     #print( arrived_ids)
     for arrive in arrived_ids:
      #logging.debug("arrive::::(%s)" % (arrive))
      
      route= dict_edge_deparr[arrive] 
      destination= route[-1]
      if destination == road:
       X_arrived_i +=1
      else:
        X_arrived_i += 0
     number_vehicles = traci.edge.getLastStepVehicleNumber(road)
     Lenght_Lane = float(traci.lane.getLength(road + '_' + '0'))
     
     if Lenght_Lane > 8:
      C_i = ((Lenght_Lane *  N_Lane) / 7.5) 
     else:     
      C_i = 0
     # for interval_tag in soup.findAll("interval"):
      # interval_begin = interval_tag["begin"]
      # interval_end = interval_tag["end"]
      # if ((str(interval_begin) == str(beginp)) and (str(interval_end) == str(endp))):

       
       # #logging.debug("interval_begin::::(%s)" % (interval_begin))
       
       # #logging.debug("interval_end::::(%s)" % ((interval_end)))
       # #logging.debug("endp::::(%s)" % (endp))
       # #logging.debug("beginp::::(%s)" % ((beginp)))
       # #logging.debug("interval_begin::::(%s)" % ((interval_begin)))
       # #logging.debug("interval_begin is beginp::::(%s)" % (str(interval_begin) == str(beginp)))
       # edge_tag = interval_tag.findAll("edge")
       # #break
     
     # if road in edge_tag:
       # X_departed_i = edge["departed"]
       # X_arrived_i = edge["arrived"]
      
     # else:
      # X_departed_i =0
      # X_arrived_i=0
       
     # #logging.debug("dict_edge_deparr[road]::::(%s)" % (dict_edge_deparr[road]))
     # # 
     # # 
     # # entered , left = dict_edge_enterleft[road]
     #print(list(bfs_edges(graph, road, reverse=True, L= 1)))
     # list_succ = []
     # for successor_road in graph.successors_iter(road):
     # list_succ.append(successor_road)
     bfs_edges(graph, road, reverse=True, L= 1)
     Totaln=0
     Q_i_in=0
     dict_numVehicle_for_edge={}
     if len(visit_bfs) > 1:
      for e in visit_bfs:
       n=0
       numberV_edge_v=0
       dict_numVehicle_for_edge[e]=0
       if e==road: continue
       if e.startswith(":"): continue
       numberV_edge_v = traci.edge.getLastStepVehicleNumber(e)
       Vehicle_ids = traci.edge.getLastStepVehicleIDs(e)
       for v in Vehicle_ids:
        route = traci.vehicle.getRoute(v)
        currentpos = traci.vehicle.getRoadID(v)
        #logging.debug("currentpos::::(%s)" % (currentpos))
        if currentpos != route[-1]:
          next_road = (route.index(currentpos)) + 1
          #logging.debug("next_road::::(%s)" % (next_road))
          if route[next_road] == road:
             n += 1
       dict_numVehicle_for_edge[e]= n
       Totaln += numberV_edge_v
      nextNodeID = net.getEdge(road).getToNode().getID()
      #logging.debug("nextNodeID::::(%s)" % (nextNodeID))
      
      #logging.debug("roadroad::::(%s)" % (road))
      ##logging.debug("edgeedgeedge::::(%s)" % (edge_net))
      # connection = edge.getOutgoing()
      # #logging.debug("connection::::(%s)" % (connection))
     
       
      #logging.debug("visit_bfs::::(%s)" % (visit_bfs))
      
      
      
       
      if Totaln != 0 :
       for edge in visit_bfs:
        if edge==road: continue
        if edge.startswith(":"): continue
        numberV_edge = traci.edge.getLastStepVehicleNumber(edge)
        edge_nei = net.getEdge(edge)
        TLS = edge_nei.getTLS()
        time = edge_nei.getLength() / edge_nei.getSpeed()
        if TLS is not None:
         
             tlane = graph[edge][road]["TLane"]
             flane = graph[edge][road]["FLane"]
             TLSID= TLS.getID()
             links = traci.trafficlight.getControlledLinks(TLSID)
             remaningdur = (traci.trafficlight.getNextSwitch(TLSID)- traci.simulation.getCurrentTime()) /1000
             state = traci.trafficlight.getRedYellowGreenState(TLSID)
             dict_State ={}
             i = 0
             for sta in state:
              dict_State[i] = sta
              i += 1
              #logging.debug("sta: %s" % sta)
             for conn in links:
              indicate = links.index(conn)
              #logging.debug("indicate: %s" % indicate)
              if not conn:
                continue
              if (edge + '_' + flane)== conn[0]:
               if (road + '_' + tlane)== conn[1]:
                if dict_State[indicate] == 'G':
                 Duration = traci.trafficlight.getPhaseDuration(TLSID)
                 Q_i_in +=  numberV_edge * (dict_numVehicle_for_edge[edge] / Totaln) *  (Duration / 90) * (1/graph[edge][road]["TTg"])
             
             
             # #logging.debug("numberV_edge::::(%s)" % (numberV_edge))
             # #logging.debug("dict_numVehicle_for_edge[edge]::::(%s)" % (dict_numVehicle_for_edge[edge]))
             # #logging.debug("Totaln::::(%s)" % (Totaln))
             
        else:
         Q_i_in += (dict_numVehicle_for_edge[edge] / Totaln) *  (1/graph[edge][road]["TTg"]) 
        
        
      else:
       Q_i_in = 0
       
     
       
      
     else:
      Q_i_in = 0#dict_numVehicle_for_edge[e]= 0
      
       
     
     # for e in visit_bfs:
      # if e==road: continue
      # if e.startswith(":"): continue
      # numberV_edge_v = traci.edge.getLastStepVehicleNumber(e)
      # Vehicle_ids = traci.edge.getLastStepVehicleIDs(e)
      # n= 0
      
      # for v in Vehicle_ids:
       # route = traci.vehicle.getRoute(v)
       # currentpos = traci.vehicle.getRoadID(v)
       # #logging.debug("currentpos::::(%s)" % (currentpos))
       # if currentpos != route[-1]:
         # next_road = (route.index(currentpos)) + 1
         # #logging.debug("next_road::::(%s)" % (next_road))
         # if route[next_road] == road:
           # n += 1
       # # else:
        # # if route[next_road] == road:
         # # n += 1
        
      # dict_numVehicle_for_edge[e]= n
      # Totaln += numberV_edge_v
        
     # if len(visit_bfs) > 1:
      # for edge in visit_bfs:
       # if edge==road: continue
       # if edge.startswith(":"): continue
       # numberV_edge = traci.edge.getLastStepVehicleNumber(edge)
       # if Totaln != 0 :
             # Q_i_in += numberV_edge * (dict_numVehicle_for_edge[edge] / Totaln)

       # else:
          # Q_i_in += 0
     # else:
      # Q_i_in += 0
      
     
     ###############################################################
     #print(list(bfs_edges(graph, road, reverse=False, L= 1)))
     bfs_edges(graph, road, reverse=False, L= 1)
     nn = 0
     Totalnn = 0
     Q_i_out = 0
     dict_numVehicle_for_next_road={}
     
     
     
     listOutfromgoing = net.getEdge(road).getToNode().getIncoming()
     #logging.debug("listOutfromgoing: %s" % listOutfromgoing)
     EdgeId_Outfromgoing=[]
     for edgelist in listOutfromgoing:
        list_edgelist=[]
        list_edgelist.append(str(edgelist))
        data = list_edgelist[0].split('id=')[1]
        EdgeId_Outfromgoing.append( ast.literal_eval(data.split(' ')[0]))
        #logging.debug("EdgeIdEdgeId: %s" % EdgeId_Outfromgoing)
        

    
       
              
     Vehicle_ids= traci.edge.getLastStepVehicleIDs(road)
     #
     for edge in visit_bfs:
       dict_numVehicle_for_next_road[edge] =0
     for v in Vehicle_ids:
       route= traci.vehicle.getRoute(v)
       currentpos = traci.vehicle.getRoadID(v)
       # if currentpos == route[0]:
          # X_departed_i+=1
       # if currentpos == route[-1]:
          # X_arrived_i += 1
       if currentpos != route[-1]:
        next_road= (route.index(currentpos)) + 1
        if route[next_road] in visit_bfs:
         dict_numVehicle_for_next_road[route[next_road]] += 1
                 
     for edge in EdgeId_Outfromgoing:
         Totalnn += traci.edge.getLastStepVehicleNumber(edge)
         # Vehicle_ids= traci.edge.getLastStepVehicleIDs(edge)
         # for v in Vehicle_ids:
           # route= traci.vehicle.getRoute(v)
           # currentpos = traci.vehicle.getRoadID(v)
           # next_road= (route.index(currentpos)) + 1
           # #if next_road == route[-1
           # if next_road == road:
           # nn += 1
         
         # Totalnn += nn 
    
       
       
      # #logging.debug("TLSTLSTLS::::(%s)" % (TLSID))  
     TLS = edge_road.getTLS()
     if  TLS is not None:
       TLSID= TLS.getID()
       lanes = traci.trafficlight.getControlledLanes(TLSID)
       links = traci.trafficlight.getControlledLinks(TLSID)
       ##logging.debug("TLSID::::(%s)" % (TLSID))
       ##logging.debug("road::::(%s)" % (road))
       ##logging.debug("laneslanes::::(%s)" % (lanes))
       ##logging.debug("links::::(%s)" % (links))
       phaseDuration = traci.trafficlight.getPhaseDuration(TLSID)
       phase = traci.trafficlight.getPhase(TLSID)
       state = traci.trafficlight.getRedYellowGreenState(TLSID)
       ##logging.debug("state::::(%s)" % (state))
       ##logging.debug("phase::::(%s)" % (phase))
       ##logging.debug("phaseDuration::::(%s)" % (phaseDuration))
       getSwitch = traci.trafficlight.getNextSwitch(TLSID)
       ##logging.debug("getSwitch::::(%s)" % (getSwitch))
       remaningdur = (traci.trafficlight.getNextSwitch(TLSID)- traci.simulation.getCurrentTime()) /1000

       ##logging.debug("remaningdur::::(%s)" % (remaningdur))
      #Program = traci.trafficlight.getProgram(TLSID)
       dict_State ={}
       i = 0
       for sta in state:
        dict_State[i] = sta
        i += 1
        #logging.debug("sta: %s" % sta)
     time = edge_road.getLength() / edge_road.getSpeed()  
     for edge in visit_bfs:
       if edge.startswith(":"): continue
       if edge==road:continue  
       if Totalnn !=0 :
         if TLS is not None:
          flane = graph[road][edge]["FLane"]
          tlane = graph[road][edge]["TLane"]
          ##print(links)
          for conn in links:
           indicate = links.index(conn)
           #logging.debug("indicate: %s" % indicate)
           if not conn:
             continue
           if (road + '_' + flane) == conn[0]:
            if (edge + '_' + tlane) == conn[1]:
             if dict_State[indicate] == 'G':
              Duration = traci.trafficlight.getPhaseDuration(TLSID)
              Q_i_out += number_vehicles * (dict_numVehicle_for_next_road[edge] / Totalnn) *  (Duration / 90) * (1/graph[road][edge]["TTg"]) 
              
             
                 
         else:
          #logging.debug("edgeedge::::(%s)" % (edge))
          #logging.debug("roadroadroad::::(%s)" % (road))
          
          Q_i_out += (dict_numVehicle_for_next_road[edge] / Totalnn) * (1/graph[road][edge]["TTg"]) 
       else:
          Q_i_out =0
     
     # this road all vehcle + in flow rate + in veh_num - out flow rate - out veh_num  
     X_i_calculated_next_k = max ((number_vehicles +  Q_i_in + X_departed_i - Q_i_out - X_arrived_i), 0)
     
     for successor_road in graph.successors(road):
        if C_i != 0 :
         x = X_i_calculated_next_k / C_i
         if x > 1 :
          x= 1
         
         graph[road][successor_road]["predict"] = x
        else:
         graph[road][successor_road]["predict"] = 0
        #logging.debug("predictpredictpredictpredict: %s" % graph[road][successor_road]["predict"])
        #logging.debug("X_i_calculated_next_k: %s" % X_i_calculated_next_k)
        #logging.debug("number_vehicles: %s" % number_vehicles)
        #logging.debug("Q_i_in: %s" % Q_i_in)
        #logging.debug("X_departed_i: %s" % X_departed_i)
        #logging.debug("Q_i_out: %s" % Q_i_out)
        #logging.debug("X_arrived_i: %s" % X_arrived_i)
        #logging.debug("C_i: %s" % C_i)

        list_predict.append(graph[road][successor_road]["predict"])
        d= graph[road][successor_road]["predict"]
        #print("roadtest: {0}".format(road))
        #print ("{0}w6 id={1}{2}{3} {4}{5}" .format('<','"',d,'"','/','>'))

  
def Road_RSU_congestion_index(edgelist,rsuid, net): # Zone Congestion
             list_of_vehicle=[]
             congestedRoads=[]
             vehiclecongested=[]
             Sum_Zone_RSUs={}
             Sum_Zone=0
             Sum_Lane_L=0
             Sum_Lane_WedgeZ=0
             for edge in edgelist[rsuid]:
                  lanelen = traci.lane.getLength(edge + '_' + '0')
                  Max_speed_LaneZ = traci.lane.getMaxSpeed(edge + '_' + '0')
                  MeanSpeedZ= traci.edge.getLastStepMeanSpeed(edge)

                  edge_road = net.getEdge(edge)
                  N_Lane = edge_road.getLaneNumber()
                  #logging.debug("N_Lane[edge]::::(%s)" % (N_Lane))
                  Kjam = float ((lanelen * N_Lane)/ (7.5))
                  number_vehicles = traci.edge.getLastStepVehicleNumber(edge)
                  Wedgev = 1 - (MeanSpeedZ / Max_speed_LaneZ)
                  Wedgek = float(number_vehicles / Kjam)

                  if Wedgev <0:
                      Wedgev = 0
                  if Wedgev >1:
                      Wedgev = 1
                  if Wedgek <0:
                      Wedgek = 0
                  if Wedgek > 1:
                     Wedgek= 1

                  WedgeZ = float((Wedgev + Wedgek) / (2))
                  Sum_Lane_L += lanelen 
                  Sum_Lane_WedgeZ  += lanelen * WedgeZ
             Sum_Zone = float(Sum_Lane_WedgeZ  / Sum_Lane_L )
             #logging.debug("Sum_ZoneSum_Zone: (%s, %s)" % (Sum_Zone, rsuid))
             return float(Sum_Zone)


def travel_time_on_roads(graph,time, begin_of_cycle):
    global edgListspeed
    global edgListLength
    global dict_fc
    global listtraveltime
    Lenght_Lane = 0
    max_speed_Lane = 0
    travel_time = 0
    avespeed = 0
    maxLenght = 0
    mul_max = 0
    mean = 0
    Vmax = 0
    vmaxlane = 0.0
    Vmax = max(edgListspeed) 
    maxLenght = max(edgListLength)
    mul_max= float(maxLenght * Vmax)
    maxtraveltime = 0.0
    for road in graph.nodes():
        travel_time = traci.edge.getAdaptedTraveltime(road, time)
        travel_timenon = traci.edge.getAdaptedTraveltime(road, time)
        #logging.debug("AdaptedTraveltime::::(%s)" % (travel_time))
        vmaxlane = traci.lane.getMaxSpeed(road + '_' + '0')
        dict_fc[road]= traci.edge.getLastStepVehicleNumber(road)
        #logging.debug("maxLenght :::: (%s)" % maxLenght)
        if travel_time <= 0:
            Lenght_Lane = float(traci.lane.getLength(road + '_' + '0')) 
            Vcurrent = traci.edge.getLastStepMeanSpeed(road)
            #logging.debug("Vcurrent::::(%s)" % (Vcurrent))
            #logging.debug("Lenght::::(%s)" % (Lenght_Lane ))
            #logging.debug("Vmaxlane :::: (%s)" % vmaxlane)
            travel_timenon = traci.edge.getTraveltime(road)
            travel_time = float((Lenght_Lane) / ((Vcurrent + vmaxlane)* maxLenght) )
        for successor_road in graph.successors(road):
            if begin_of_cycle:
                graph[road][successor_road]["weight"] = travel_time
                graph[road][successor_road]["TTg"] = travel_timenon
            else:
                t = float((graph[road][successor_road]["weight"]) + (travel_time)) / 2 #### t!=0
                Vcurrent = traci.edge.getLastStepMeanSpeed(road)
                t = t if t > 0 else travel_time
                graph[road][successor_road]["weight"] = t
                tt = float((graph[road][successor_road]["TTg"]) + (travel_time)) / 2
                tt = tt if tt > 0 else travel_timenon
                graph[road][successor_road]["TTg"] = tt
                #logging.debug("travel_timenotbegin:::(%s)" % (t))
                #logging.debug("Vcurrent:::-340581875#0(%s)" % (road))
                #logging.debug("Road:::-340581875#0(%s)" % (Vcurrent))
            listtraveltime.append(graph[road][successor_road]["weight"])
            d = graph[road][successor_road]["weight"]
            #print("roadtest: {0}".format(road))
            #print ("{0}w1 id={1}{2}{3} {4}{5}" .format('<','"',d,'"','/','>'))

            
            
        

def prediction_area_RSU(subgraph_g,graph, edgelist, net):
 list_temp=[]
 list_all=[]
 veh_num=0
 # Input  
 global index
 global edgeallgraph
 #f = open('data/01/od_route_file.odtrips'+str(index)+'.rou.xml', 'r')
 #data = f.read()
 #soup = BeautifulSoup(data,'lxml')
 #f.close()
 #dict_edge_deparr={}
 X_arrived_area = 0
 Q_i_out = 0
 '''
 for vehicle_tag in soup.findAll("vehicle"):
      vehicle_id = vehicle_tag["id"]
      vehicle_route = vehicle_tag.find("route")
      vehicle_edges = vehicle_route["edges"]
      dict_edge_deparr[vehicle_id]= vehicle_edges 
'''
 C_i=0
 for road in  edgelist: # Total Vehicle Number in Area
  edge_road = net.getEdge(road)
  N_Lane = edge_road.getLaneNumber()
  Lenght_Lane = float(traci.lane.getLength(road+ '_' + '0'))
  if Lenght_Lane > 8:
   C_i += ((Lenght_Lane *  N_Lane) / 7.5) 
  else:
   C_i += 0
  list_temp += traci.edge.getLastStepVehicleIDs(road)
  veh_num = veh_num + (traci.edge.getLastStepVehicleNumber(road))
 
 for v in list_temp:
          ##sys.stdout.write(v)
          route =  traci.vehicle.getRoute(v) 
          source = traci.vehicle.getRoadID(v)
          
          if source == route[-1]: 
              X_arrived_area = X_arrived_area + 1 # Total Vehicle Number arrived in next time
          else:
              next_road = (route.index(source)) + 1
              if next_road not in edgelist: 
                  Q_i_out += 1 # Total Vehicle Number out in next time
 departed_ids = traci.simulation.getDepartedIDList() 
 X_departed_area = 0
 for road in edgelist:
  for depart in departed_ids: # Total Vehicle Number departed in next time
   route = traci.vehicle.getRoute(depart)
   initialpos= route[0]
   if initialpos == road :
    X_departed_area +=1
   else:
    X_departed_area +=0
 for road in edgeallgraph:# all current vehicles in the road network
  list_all += traci.edge.getLastStepVehicleIDs(road)
 Q_i_in = 0
 for v in list_all: # Total Vehicle Number input in next time
        if v not in list_temp:
              route =  traci.vehicle.getRoute(v) 
              source = traci.vehicle.getRoadID(v)
              if source != route[-1]:  
                  next_road = (route.index(source)) + 1
                  if next_road  in edgelist: 
                      Q_i_in += 1 
     
 for road in  subgraph_g.nodes():
  if road.startswith(":"):
    for successor_road in subgraph_g.successors(road):
      graph[road][successor_road]["predictArea"]=0
  else:
   #<!-- error
   for successor_road in subgraph_g.successors(road):
       graph[road][successor_road]["predictArea"] = max ((veh_num +  Q_i_in + X_departed_area - Q_i_out - X_arrived_area), 0)
   
 
def build_road_graph(network, net):       
    global dict_lane         
    global laneallgraph
    global edgeallgraph
    global edgeallsgraph
    global edgListLength
    global edgListspeed
    f = open("data/Tainan.net.xml", 'r')
    data = f.read()
    soup = BeautifulSoup(data,'lxml')
    f.close()
    #sys.stdout = open('RSU.txt','wt')
    edges_length={}
    numberlane = {}
    mul=0
    
    for edge_tag in soup.findAll("edge"):
      edge_id = edge_tag["id"]
      lane_tag = edge_tag.find("lane")
      edge_length = float(lane_tag["length"])
      edge_speed = float(lane_tag["speed"])
      edges_length[edge_id] = edge_length
      edgListLength.append(edge_length)
      edgListspeed.append(edge_speed)
      edgeallsgraph.append(edge_id)
      lane_tagall= edge_tag.findAll("lane")
      laneid=[]
      number_of_lane[edge_id]=0
      for lane in lane_tagall:
       laneid.append(lane["id"])
       laneallgraph.append(lane["id"])
       dict_lane[edge_id]= laneid
       number_of_lane[edge_id]= number_of_lane[edge_id] + 1  
    graph = nx.DiGraph() 
    for edg_tag in soup.findAll("edge"):
     edg_id = edg_tag["id"]
     if edg_id.startswith(":") : continue
     edgeallgraph.append(edg_id)
    for connection_tag in soup.findAll("connection"):
        source_edge = connection_tag["from"]        
        dest_edge = connection_tag["to"]
        if source_edge.startswith(":") or dest_edge.startswith(":"):
         graph.add_edge(source_edge, dest_edge, length=edges_length[source_edge], TTg=0, weight=0 , TT=0, Counter=0)
        else:
         edge_net = net.getEdge(source_edge)
         nextEdge = net.getEdge(dest_edge)
         connection = str(edge_net.getOutgoing()[nextEdge][0])
         FromLane = connection.split(' ')[3]
         s,fromLane = FromLane.split("fromLane=")
         #logging.debug("connection::::(%s)" % (connection))
         #logging.debug("FromLaneFromLaneFromLane::::(%s)" % (fromLane))
         ToLane = connection.split(' ')[4]
         s,toLane = ToLane.split("toLane=")
         #logging.debug("ToLaneToLane::::(%s)" % (toLane))
         graph.add_edge(source_edge, dest_edge, length= edges_length[source_edge], FLane = fromLane, TLane = toLane, TTg=0, weight=0 ,predict=0, TT=0, Counter=0,predictArea=0)
        
    
    return graph

def run(network, begin, end, interval):###############################
    initializeData()
    global avgLengthall
    global avgSpeedall
    global edgeallgraph
    global visit_bfs
    global dict_fc
    global list_present_network
    global listcdm
    #print(edgeallgraph)
    net = sumolib.net.readNet("data/Tainan.net.xml")
    road_graph_travel_time = build_road_graph(network, net) 
    buffered_paths = {}    
    list_of_vehicle=[]
    subgraph_g={}
    dicttt_edgeRSUs={}
    f = open('RSUsLocationUBC.xml', 'r')
    data = f.read()
    soup = BeautifulSoup(data,'lxml')
    f.close()
    RSU_ids=[]
    RSU_x={}
    RSU_y={}
    edgelist={}
    closestEdge=[]
    edg_ids=[]
    listalledge=[]
    RSUedge_list = []
    for RSU_tag in soup.findAll("poly"):
      RSU_id = RSU_tag["id"]
      RSU_ids.append(RSU_id)
      RSU_center = RSU_tag["center"]
      RSU_x[RSU_id], RSU_y[RSU_id] = RSU_center.split(",")
      x =float(RSU_x[RSU_id])
      #print("x= ",x)
      y =float(RSU_y[RSU_id])
      #print("y= ",y)
      list_EdgeId=[]
      list_NormalEdges=[]
      list_junctions=[]
      edges = net.getNeighboringEdges(x, y, 240)#140
      for ege in edges:
        closestEdge , dist = ege
        list_NormalEdges.append(str(closestEdge))
        data = list_NormalEdges[0].split('id=')[1]
        EdgeId = data.split(' ')[0]   
        if EdgeId in RSUedge_list:
            del list_NormalEdges[:]
            continue
        else:
            RSUedge_list.append(EdgeId)
        listalledge.append(ast.literal_eval(EdgeId))
        list_EdgeId.append(ast.literal_eval(EdgeId))

        del list_NormalEdges[:]# : means all memebers

      
      dicttt_edgeRSUs[RSU_id]=list_EdgeId
      edgelist[RSU_id]= list_EdgeId
      del list_NormalEdges[:]
    buffered_paths = {}
    step = 1
    rerouting_step = begin #800
    interval = 300
    travel_time_cycle_begin = interval  #600
    duration = 300
    beginp = 0
    endp = 1
    periodic  = 1
    zone_conges={}
    lengthSum = 0.0
    edgeCount=0
    sum_speed_net=0
    for edge in edgeallgraph:
             sum_speed_net += traci.lane.getMaxSpeed(edge + '_' + '0')
             lengthSum += traci.lane.getLength(edge + '_' + '0')    
             edgeCount += 1         
    avgLengthall = lengthSum / edgeCount
    avgSpeedall = sum_speed_net / edgeCount
    #sys.stdout = open('TestAll.txt','wt')
    '''
    total = 0
    for a,b in edgelist.items():
        print(len(b))
        total += len(b)
    print(total)
    '''
    while step == 1 or traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        #if step >1000:
        #    break
           
        detect_vehicle_in_RSU_other=[]
        #logging.debug("Simulation time %d" % step)  
        if periodic >=300 and periodic <= end and periodic%duration==0:
           #logging.debug("Simulation Step: %d" % step)
           for rsuid in RSU_ids:
            if len(edgelist[rsuid]) > 0:
             #logging.debug("rsuid for zone_conges start %s" % rsuid)
             #logging.debug("rsuid edgelist[%s]  %s" % (rsuid,edgelist[rsuid]))
             zone_conges[rsuid] = float(Road_RSU_congestion_index(edgelist, rsuid, net))
             subgraph_gg=road_graph_travel_time.subgraph(dicttt_edgeRSUs[rsuid])
             #print("enter")
             prediction_area_RSU(subgraph_gg,road_graph_travel_time, edgelist[rsuid], net)
           #print("leave !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
            
             #logging.debug("rsuid for zone_conges end %s" % zone_conges[rsuid])
        
        if step >= travel_time_cycle_begin and travel_time_cycle_begin <= end and step%interval == 0:
         #print("enter2")
         periodic  = 1
         # #logging.debug("Updating travel time on roads at simulation time %d" % step)
         list_present_network=[]
         ################ Update Travel Time ##############################
         #print("first")
         travel_time_on_roads(road_graph_travel_time, step, travel_time_cycle_begin == step)
         prediction_on_edge(road_graph_travel_time, net)
         road_congestion_at_traffic_area(road_graph_travel_time, edgelist, zone_conges, RSU_ids,net)

         #print("out")
         #update_travel_time_on_roads(road_graph_travel_time, time,  travel_time_cycle_begin == step)
         Update_Travel_Time(road_graph_travel_time, net)
         for edge in edgeallgraph:
             list_present_network.append(traci.edge.getLastStepVehicleIDs(edge))
         '''   
         for vehicle in list_present_network:#list_vehicle_set_route:
               for v in vehicle :# if vehicle in list_present_network:
                    source = traci.vehicle.getRoadID(v)
                    dict_fc[source] = dict_fc[source] + 1
         '''
    #########################################################################################################################################        
       ########################################## select congested Roads for rerouting by cloud server####################################       

         congestedRoadsss=[]
         for edge in edgeallgraph:
         #for lanelist in dict_lane[edge]:
            #for lane in lanelist:
               listspeedss=[]
               listvss=[]
               Max_speed_LaneC = traci.lane.getMaxSpeed(edge + '_' + '0')# Get Maximum Allowed speed
               listvss.append(traci.edge.getLastStepVehicleIDs(edge))
               MeanSpeeds= traci.edge.getLastStepMeanSpeed(edge)
               sumspeedss=0
               averagespeedss=0
               jjj=0
               wwwedge=0
               listvehicless=[]
               vehiclecongestedss=[]
               Wedge_cloud = 1-(MeanSpeeds / Max_speed_LaneC)
               #logging.debug("Wedge_cloud: %s" % Wedge_cloud)
               Denc=0
               edge_road = net.getEdge(edge)
               N_Lane = edge_road.getLaneNumber()
               number_vehicles =(traci.edge.getLastStepVehicleNumber(edge))
               Lenght_Lane = traci.lane.getLength(edge + '_' + '0')
               Kjam = float ((Lenght_Lane * N_Lane)/ (7.5))
               if number_vehicles !=0 and Lenght_Lane >= 9:
                    Density = float (number_vehicles / Kjam)
               else:
                    Density = 0.0

               if Wedge_cloud < 0:
                 Wedge_cloud = 0
               if  Wedge_cloud > 1:
                  Wedge_cloud = 1
               if Density < 0:
                 Density = 0
               if Density > 1:
                 Density = 1 
               if (  Wedge_cloud  +   Density  ) / 2 >= 0.5:
                 congestedRoadsss.append(edge)
         #############################################  select vehicle for rerouting by cloud server  #######################################       
         list_of_vehiclecloud=[]
         for edge in edgeallgraph:
             if edge in listalledge:continue
             list_of_vehiclecloud.append(traci.edge.getLastStepVehicleIDs(edge)) 
         #print("out2")              
         if len(congestedRoadsss) != 0 :
            #logging.debug("cloud_reroute_vehicles is start: " )  
            cloud_reroute_vehicles(road_graph_travel_time,list_of_vehiclecloud,congestedRoadsss,buffered_paths, travel_time_cycle_begin == step, net)
            #logging.debug("cloud_reroute_vehicles is end: " )
         #######################################################  Re-Routing for each RSU ################################################
            list_of_vehicle=[]
            for rsuid in RSU_ids:
             subgraph_g[rsuid]=road_graph_travel_time.subgraph(dicttt_edgeRSUs[rsuid])
             
             congestedRoads=[]
             vehiclecongested=[]
             list_present=[]
             if len(edgelist[rsuid])== 0:continue
             for edge in edgelist[rsuid]:
                   #del visit_bfs[:]
                   #logging.debug("edgelist (%s, %s)" % (rsuid, edge))
                   MeanSpeed=0.0
                   number_vehicle=0 
                   max_speed_Lane = float(traci.lane.getMaxSpeed(edge + '_' + '0'))# Get Maximum Allowed speed
                   Lenght_Lane = float(traci.lane.getLength(edge + '_' + '0'))
                   LastStepLength = float(traci.edge.getLastStepLength(edge))
                   listv=traci.edge.getLastStepVehicleIDs(edge) # Get the all vehicle in lane
                   MeanSpeed= float(traci.edge.getLastStepMeanSpeed(edge))
                   Wedge = float(1 - (MeanSpeed / max_speed_Lane))
                   Denc=0
                   edge_road = net.getEdge(edge)
                   N_Lane = edge_road.getLaneNumber()
                   number_vehicles =(traci.edge.getLastStepVehicleNumber(edge))
                   Kjam = float ((Lenght_Lane * N_Lane)/ (7.5))
                   if number_vehicles !=0 and Lenght_Lane >= 9:
                    Density = float (number_vehicles / Kjam)
                   else:
                    Density = 0.0

                   if Wedge <0:
                     Wedge = 0
                   if Wedge >1:
                     Wedge = 1
                   if Density <0:
                     Density = 0
                   if Density > 1:
                     Density = 1
                   
                   ZoneC = zone_conges[rsuid]
                   sum_conges= (Wedge+ZoneC+Density)/3
                   listcdm.append(sum_conges)    
                   if ZoneC >=0.5:
                    #<!-- error
                    sum_conges= (Wedge+ZoneC+Density)/3
                   else:
                    sum_conges= ( Wedge + Density )
                   
                   if sum_conges >= 0.5:#if  (( Wedge )+ ( Density )) / 2 >= 0.5: # sum_conges >= 0.5:# (Wedge+Density)/2 >= 0.5: # ((sum_conges + ZoneC)/ 2) >= 0.5: 
                        #print(list(bfs_edges(subgraph_g[rsuid], edge, reverse=True, L=3)))
                        bfs_edges(subgraph_g[rsuid], edge, reverse=True, L=3)
                        #print("88888")
                        congestedRoads.append(edge)
                        #print(visit_bfs)
                        for v in visit_bfs:
                            if v in list_present: continue
                            list_present.append(v)
                            if v.startswith(":"): continue
                            vehiclecongested.append(traci.edge.getLastStepVehicleIDs(v))
                   for vv in vehiclecongested:
                       for v in vv:
                           if v not in list_of_vehicle:
                               list_of_vehicle.append(v)
                               #print('enter')
                   '''         
                   if len(detect_vehicle_in_RSU_other)!= 0:
                      #print("333")
                      for vv in vehiclecongested:
                        for v in vv:
                          if v not in detect_vehicle_in_RSU_other:
                           list_of_vehicle.append(v)
                           detect_vehicle_in_RSU_other.append(v)
                            
                   else:
                      #print("entrt")

                      for vv in vehiclecongested:
                        #print(vv)
                        for v in vv:
                          list_of_vehicle.append(v)
                          #print(len(list_of_vehicle)) 
                          detect_vehicle_in_RSU_other.append(v)    
                    '''     
             if len(congestedRoads) > 0 :
               #logging.debug("Simulation for  %s" % rsuid)
               #logging.debug("Simulation time %d" % step)
               #logging.debug("congestedRoads %s" % congestedRoads)
               #logging.debug("list_of_vehicle %s" % list_of_vehicle)
               #logging.debug("detect_vehicle_in_RSU_other: %s" % detect_vehicle_in_RSU_other)
               reroute_vehicles_RSUs(subgraph_g[rsuid],list_of_vehicle,edgelist[rsuid],congestedRoadsss,congestedRoads,road_graph_travel_time,buffered_paths, travel_time_cycle_begin == step,net)
              
            rerouting_step += interval
            travel_time_cycle_begin = step + 1
        step += 1 
        periodic +=1
   
    #time.sleep(10)
    #logging.debug("Simulation finished")
    traci.close()
    #sys.stdout.flush()
    #time.sleep(10)
    

class UnusedPortLock:
    lock = thread.allocate_lock()

    def __init__(self):
        self.acquired = False

    def __enter__(self):
        self.acquire()

    def __exit__(self):
        self.release()

    def acquire(self):
        if not self.acquired:
            UnusedPortLock.lock.acquire()
            self.acquired = True

    def release(self):
        if self.acquired:
            UnusedPortLock.lock.release()
            self.acquired = False

def find_unused_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
    sock.bind(('127.0.0.1', 0))
    sock.listen(socket.SOMAXCONN)
    ipaddr, port = sock.getsockname()
    sock.close()
    
    return port
def terminate_sumo(sumo):
    if sumo.returncode == None:
        os.kill(sumo.pid, signal.SIGTERM)
        time.sleep(0.5)
        if sumo.returncode == None:
            #print(os.__file__)
            #os.kill(sumo.pid, signal.SIGKILL)
            time.sleep(1)
            if sumo.returncode == None:
                time.sleep(10)
    
def start_simulation(sumo, scenario, network, begin, end, interval, output):
    #logging.debug("Finding unused port")
    #print("Finding unused port")
    
    sumoBinary = checkBinary('sumo')
    for i in range(0,15):
        #unused_port_lock = UnusedPortLock()
        #unused_port_lock.__enter__()
        #remote_port = find_unused_port()
        ##print("remote_port:{0}" .format(remote_port))
        ##logging.debug("Port %d was found" % remote_port)
        global index
        index = i + 1
        #logging.debug("Starting SUMO as a server")
        if i < 9:
            pn = "data/01"+"/re0"+str(i+1)+".sumocfg"
            fn = "Experiment_result/tripinfo"+str(i+1)+".xml"
        else:
            pn = "data/01"+"/re"+str(i+1)+".sumocfg"
            fn = "Experiment_result/tripinfo"+str(i+1)+".xml"
        #sumo = subprocess.Popen([sumoBinary, "-c", pn , "--tripinfo-output", fn,"--device.emissions.probability", "1.0",  "--remote-port", str(remote_port)], stdout=#sys.stdout, stderr=sys.stderr) 
        #sumo = subprocess.Popen([sumoBinary, "-c", pn , "--tripinfo-output", fn,"--device.emissions.probability", "1.0"], stdout=#sys.stdout, stderr=sys.stderr)    
        #unused_port_lock.release()
        traci.start([sumoBinary, "-c", pn,"--tripinfo-output", fn])
        run(network, begin, end, interval)
        try:     
            #traci.init(remote_port)    
            pass
        except Exception:
            print("something wrong")
            #logging.exception("Something bad happened")
        finally:
            pass
            #logging.exception("Terminating SUMO")  
            #terminate_sumo(sumo)
            #unused_port_lock.__exit__()
def main():
    # Option handling
    parser = OptionParser(conflict_handler="resolve")#parser = OptionParser()
    parser.add_option("-c", "--command", dest="command", default="sumo", help="The command used to run SUMO [default: %default]", metavar="COMMAND")
    parser.add_option("-s", "--scenario", dest="scenario", default="Your.sumo.cfg", help="A SUMO configuration file [default: %default]", metavar="FILE")
    parser.add_option("-n", "--network", dest="network", default="Your.net.xml", help="A SUMO network definition file [default: %default]", metavar="FILE")    
    parser.add_option("-b", "--begin", dest="begin", type="int", default=800, action="store", help="The simulation time (s) at which the re-routing begins [default: %default]", metavar="BEGIN")
    parser.add_option("-a", "--additional-files", dest="additional", default="Your.add.xml", help="Generate edge-based dump instead of ""lane-based dump. This is the default.", metavar="FILE")
    parser.add_option("-a", "--additional-files", dest="additional", default="Your.add.xml", help="Generate edge-based dump instead of ""lane-based dump. This is the default.", metavar="FILE")
    parser.add_option("-e", "--edge-based-dump", dest="edge_based_dump", action="store_true", default="True", help="Generate edge-based dump instead of ""lane-based dump. This is the default.", metavar="FILE")
    parser.add_option("-e", "--end", dest="end", type="int", default=72000, action="store", help="The simulation time (s) at which the re-routing ends [default: %default]", metavar="END")
    parser.add_option("-i", "--interval", dest="interval", type="int", default=600, action="store", help="The interval (s) at which vehicles are re-routed [default: %default]", metavar="INTERVAL")
    parser.add_option("-o", "--output", dest="output", default="Your.xml", help="The XML file at which the output must be written [default: %default]", metavar="FILE")
    parser.add_option("-l", "--#logfile", dest="#logfile", default=os.path.join(tempfile.gettempdir(), "SUMO_ReFOCUS.#log"), help="#log messages to #logfile [default: %default]", metavar="FILE")
    (options, args) = parser.parse_args()
    #logging.basicConfig(filename=options.#logfile, level=#logging.DEBUG)
    #logging.debug("#logging to %s" % options.#logfile)
    if args:
        pass
        #logging.warning("Superfluous command line arguments: \"%s\"" % " ".join(args))
    start_simulation(options.command, options.scenario, options.network, options.begin, options.end, options.interval, options.output)
if __name__ == "__main__":
    #print("12222")
    main()    
    
 