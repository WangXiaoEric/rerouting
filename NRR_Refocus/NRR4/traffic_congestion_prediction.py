# -*- coding: utf-8 -*-
"""
Created on Tue Jun  2 06:27:56 2020

@author: user
"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

#from sumolib import checkBinary  # noqa
import traci

"""
update densities of every RS in road network
RS_info = {RS_id:[from(node),to(node),lenght,lane_number]
"""
def updateRSDensities(RS_info, RSNumber):
    
    #VehicleNumber = [[0]*5 for i in range(RSNumber)]
    #i = 0
    VehicleNumber = {}
    """
    VehicleNumber:
    0:id;
    1:vehicle number;
    2:length;
    3:lane_number;
    """
    for RS_id,info_list in RS_info.items():
        VehicleNumber[RS_id ] ={}
        VehicleNumber[RS_id ]["Veh_Num"] = str(traci.edge.getLastStepVehicleNumber(RS_id))
        VehicleNumber[RS_id ]["Length"] = info_list[2]
        VehicleNumber[RS_id ]["Lane_Num"] = info_list[3]
        VehicleNumber[RS_id ]["Lane_ID"] = info_list[4]
        '''
        VehicleNumber[i][0] = RS_id
        VehicleNumber[i][1] = str(traci.edge.getLastStepVehicleNumber(RS_id))
        VehicleNumber[i][2] = info_list[2]
        VehicleNumber[i][3] = info_list[3]
        VehicleNumber[i][4] = info_list[4]
        i += 1
        '''
    
    return VehicleNumber

def detectCongestion(RSDensities, con_threshold,edgeocc_dict):
    
    congestedRS = {}
    content = {}
    SimTime = int(traci.simulation.getTime())
    for RS in RSDensities:
        
        """ get mean speed """
        mean_speed = float(traci.edge.getLastStepMeanSpeed(RS))
        #print(mean_speed )
        """ get speed limit """
        speed_limit = float(traci.lane.getMaxSpeed( RSDensities[RS]["Lane_ID"] ))
        
        """ N_max = Length(RS)*Lane_num(RS)/(avg vehicle length + min gap between vehicles) """
        N_max = float(RSDensities[RS]["Length"])*float(RSDensities[RS]["Lane_Num"])/(5.0+2.5)
        
        """occupancy"""
        occ = float(RSDensities[RS]["Veh_Num"])/N_max
        
        line = str(occ)+","+str(float(RSDensities[RS]["Veh_Num"]))+","+str(float(RSDensities[RS]["Length"]))+","+str(float(RSDensities[RS]["Lane_Num"]))
        edgeocc_dict[RS][SimTime] = line
        """velocity"""
        vel = 1-mean_speed/speed_limit
        if occ > 1:
            occ = 1
        if vel > 1:
            vel = 1
        
        """ current information N(RS)/N_max(RS)"""
        current_info = 0.5*(occ + vel)
        
        content.update({RS[0]:[occ, vel]})
        
        if occ > con_threshold :#and vel > con_threshold:
            #print("---------------------")
            #traci.edge.getStreetName(RS)
            #print(float(RSDensities[RS]["Veh_Num"]))
            #print(mean_speed)
            #print("---------------------")
            congestedRS.update({RS:current_info})
            
    return congestedRS, content, edgeocc_dict



