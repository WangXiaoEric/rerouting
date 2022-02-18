# -*- coding: utf-8 -*-
import traci
import scipy.stats as st
import numpy as np

"""   
pick the optimal route from several candidates
"""
def pickPath(vehicle, K_paths, speed_result, RS_info, TL_info, conn_TL, Coordinate, RSDensities, RS_from, RS_to, predicted_speeds):
    
    newPath = []
    travel_time = []
    path_cost = {}
    #Returns the current simulation time in s
    current_time = traci.simulation.getTime()
    
    for path in K_paths:
        
        gd = 0
        tt = 0
        total_ro = 0
        depart_time = current_time
        #print("----------------")
        for idx, RS in enumerate(path):
            #get RS_Length and RS_Kjam
            RS_Length = float(RS_info[RS][2])

            #first RS: minus the length that already passed on the first RS
            if idx == 0:
                RS_Length = RS_Length - traci.vehicle.getLanePosition(vehicle)
            
            pred_avg = speed_result[RS]
            RS_passing_time = RS_Length / pred_avg#predicted_speed

            tt = tt + RS_passing_time
            depart_time += RS_passing_time

        travel_time.append(tt)


    for i in range(len(travel_time)):
        path_cost.update({travel_time[i]:K_paths[i]})

    ## sort by key (travel time)
    path_cost = {k: v for k, v in sorted(path_cost.items(), key=lambda item: item[0])} 
    
    
    for route in path_cost.values():
        
        try:
            
            traci.vehicle.setRoute(vehicle, route)
            newPath = route        
            break         

        except:
            pass
            #print("Route replacement failed for ",vehicle)

    return newPath , predicted_speeds

   
