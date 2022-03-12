# -*- coding: utf-8 -*-
import traci
import scipy.stats as st
import numpy as np
from Prediction_model import prediction_model as pm
import random

"""   
pick the optimal route from several candidates
"""
def pickPath(vehicle, K_paths, five_minu_loopd_avg_speed_result, RS_info,
             TL_info, conn_TL, Node_Coordinate, RSDensities, RS_from, RS_to, Model_dict, predicted_speeds, MeanSpeed_dict, MeanZ_dict):
    
    newPath = []
    travel_time = []
    path_cost = {}
    #Returns the current simulation time in s
    current_time = traci.simulation.getTime()
    # 張耀元部分  累積概率密度
    Bd = st.norm.cdf(traci.vehicle.getSpeedFactor(vehicle), loc = 0.95, scale = 0.1)
    for path in K_paths:
        
        gd = 0
        tt = 0
        total_ro = 0
        depart_time = current_time
        for idx, RS in enumerate(path):

            #get RS_Length and RS_Kjam
            RS_Length = float(RS_info[RS][2])

            #first RS: minus the length that already passed on the first RS
            if idx == 0:
                RS_Length = RS_Length - traci.vehicle.getLanePosition(vehicle)

            #熱啟動 先臨時給初始數值
            if Model_dict == None:
                # pred_avg = traci.edge.getLastStepMeanSpeed(RS)
                pred_avg = random.uniform(0.1, 13.89)
            else:
                #TODO 这部分如果是None该怎么办 再用预测数值 depart_time其實為目標預測時間
                pred_avg , predicted_speeds = pm.getSpeed(RS, five_minu_loopd_avg_speed_result, current_time, depart_time, Model_dict[RS], predicted_speeds, MeanSpeed_dict[RS], MeanZ_dict[RS], Bd)

            RS_passing_time = RS_Length / pred_avg #predicted_speed

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

   
