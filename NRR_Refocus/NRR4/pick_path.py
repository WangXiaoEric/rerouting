# -*- coding: utf-8 -*-
import traci
import scipy.stats as st
import numpy as np

def Cosine(dataA,dataB):
    a = np.array(dataA)[0]
    b = np.array(dataB)[0]
    denom = np.linalg.norm(a) * np.linalg.norm(b)
    return np.dot(a,b) / denom + 1

def Route_Occupancycost(ro , length, idx):
    total = (1 + length)*length/2
    weight = (length - idx +1) / total

    return weight * ro
    
def Normalize(data_list):
    min_data = min(data_list)
    max_data = max(data_list)
    if max_data == min_data:
        for idx , data in enumerate(data_list):
            data_list[idx] = 0.5
        return data_list
        
    for idx , data in enumerate(data_list):
        normalize_data = (data - min_data) / (max_data - min_data)
        data_list[idx] = normalize_data
        
    return data_list
        
        

"""   
pick the optimal route from several candidates
"""
def pickPath(vehicle,GDpath_list, speed_result, RS_info, TL_info, conn_TL, Coordinate, RSDensities, RS_from, RS_to,NextRoad_dict):
    
    newPath = []
    road_occupancy = []
    travel_time = []
    geographic_distance = []
    start_node = []
    end_node = []
    path_cost = {}
    #Returns the current simulation time in s
    current_time = traci.simulation.getTime()
    #print(len(GDpath_list))
    for path in GDpath_list:
        
        gd = 0
        tt = 0
        total_ro = 0
        depart_time = current_time
        
        for idx, RS in enumerate(path):

            distance = float(RS_info[RS][2])
            gd = gd + distance

            if idx == 0:
                sn = RS_info[RS][0]
                en = RS_info[RS][1]

                N_max = float(RSDensities[RS]["Length"])*float(RSDensities[RS]["Lane_Num"])/(5.0+2.5)
                ro = float(RSDensities[RS]["Veh_Num"])/N_max
                if ro > 1:
                    ro = 1
                    
                total_ro += ro
                
                ro_factor = ( 1 - total_ro )
                if ro_factor == 0:
                    ro_factor = 0.0000000001
                #speed = speed_result[RS][-1]
                #if speed == 0:
                #    speed = 0.001
                tt = distance / ( 13.89 * ro_factor )
            


            
        geographic_distance.append(gd)    
        road_occupancy.append(total_ro)
        start_node.append(sn)
        end_node.append(en)
        travel_time.append(tt)

        
    cong_start = [RS_from]*len(GDpath_list)
    cong_end = [RS_to]*len(GDpath_list)
    
    start_xy = []
    end_xy = []
    cstart_xy = []
    cend_xy = []
    for i in start_node:
        start_xy.append((np.mat(Coordinate[i])).astype(float))
    for i in end_node:
        end_xy.append((np.mat(Coordinate[i])).astype(float))
    for i in cong_start:
        cstart_xy.append((np.mat(Coordinate[i])).astype(float))
    for i in cong_end:
        cend_xy.append((np.mat(Coordinate[i])).astype(float))
    
    Congestion_Closeness = []    
    for i in range(len(GDpath_list)):
        next_road = end_xy[i]-start_xy[i]
        congestion_road = cend_xy[i]-cstart_xy[i]
        similarity = (Cosine(next_road, congestion_road))
        Congestion_Closeness.append(similarity)
        
        
    def CV(data):
        mean = np.mean(data)
        std = np.std(data)
        if mean == 0:
            result = 0.0
        else:
            result = std/mean
        return result
    '''
    print(road_occupancy)
    print(travel_time)
    print(geographic_distance)
    print(Congestion_Closeness)
    print("---------------------------")
    '''
    cv = [CV(road_occupancy), CV(travel_time), CV(geographic_distance), CV(Congestion_Closeness)]
    
    w = []
    for i in cv:
        wi = i/(CV(road_occupancy)+CV(travel_time)+CV(geographic_distance)+CV(Congestion_Closeness))
        '''
        if (CV(road_occupancy)+CV(travel_time)+CV(geographic_distance)+CV(Congestion_Closeness)) == 0:
            print(vehicle)
            #print((CV(road_occupancy)+CV(travel_time)+CV(geographic_distance)+CV(Congestion_Closeness)))
            print(CV(road_occupancy))
            print(CV(travel_time))
            print(CV(geographic_distance))
            print(CV(Congestion_Closeness))
        '''
        w.append(wi)

    result = []
    road_occupancy = Normalize(road_occupancy)
    travel_time = Normalize(travel_time)
    geographic_distance = Normalize(geographic_distance)
    Congestion_Closeness = Normalize(Congestion_Closeness)
    
    for i in range(len(road_occupancy)):
        ri = w[0]*road_occupancy[i]+w[1]*travel_time[i]+w[2]*geographic_distance[i]+w[3]*Congestion_Closeness[i]
        ri = ri.tolist()
        result.append(ri) 
        
    
    for i in range(len(road_occupancy)):
        path_cost.update({result[i]:GDpath_list[i]})
        #print(result[i])
        #print(GDpath_list[i])
    #print(path_cost)

    ## sort by key (travel time)
    path_cost = {k: v for k, v in sorted(path_cost.items(), key=lambda item: item[0])} 
    current_edge = traci.vehicle.getRoadID(vehicle)
    
    #print("--------------")
    '''
    for route in path_cost.values():
        print([current_edge] + route)
        #print("*************")
    '''
    for route in path_cost.values():
        
        try:
            #print("selsecte-----------------------------------")
            #print(route)
            current_edge = traci.vehicle.getRoadID(vehicle)
            route = [current_edge] + route
            '''
            print("selsecte-----------------------------------")
            print(route)
            '''
            traci.vehicle.setRoute(vehicle, route)
            newPath = route
            #print("Route replacement success for ",vehicle)            
            break         

        except Exception as e:
            #print(e)
            pass
            #print("Route replacement failed for ",vehicle)

    return newPath

   
