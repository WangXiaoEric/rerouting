from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random

sys.path.append(os.path.join(os.environ["SUMO_HOME"], 'tools'))
import sumolib
NetName = "Tainan.net.xml"
Inputxt = "SensCoord.txt"
DetName = "Tainan.det.xml"
net = sumolib.net.readNet(NetName)
detectors = []
i=0
with open(Inputxt,'r') as rf:
    for line in rf:
        line = line.replace("\n","")
        xy_pos = []
        info = line.split(',')
        id = info[0]
        # Start Pos
        xy_pos.append( float(info[1]) )
        xy_pos.append( float(info[2]) )
        lane = net.getLane(id)
        pos, d = lane.getClosestLanePosAndDist(xy_pos,True)
        #first pos distance = 0
        #plus 2 can align traffic light
        pos +=2
        detectors.append(sumolib.sensors.inductive_loop.InductiveLoop("Start_"+id, id, pos))
        # Mid Pos
        xy_pos = []
        xy_pos.append( float(info[3]) )
        xy_pos.append( float(info[4]) )
        pos, d = lane.getClosestLanePosAndDist(xy_pos,True)
        detectors.append(sumolib.sensors.inductive_loop.InductiveLoop("Mid_"+id, id, pos))
        # End Pos
        xy_pos = []
        xy_pos.append( float(info[5]) )
        xy_pos.append( float(info[6]) )
        pos, d = lane.getClosestLanePosAndDist(xy_pos,True)
        #last pos distance = -1
        #minus 2 can align traffic light
        pos -=2
        detectors.append(sumolib.sensors.inductive_loop.InductiveLoop("End_"+id, id, pos))
        '''lanes = net.getNeighboringLanes(xy_pos[0], xy_pos[1], 5)
        # attention, result is unsorted
        bestLane = None
        ref_d = 9999.
        for lane, dist in lanes:
            # now process them and determine a "bestLane"
            # ...
            if dist < ref_d:
                ref_d = dist
                bestLane = lane
            pos, d = bestLane.getClosestLanePosAndDist(xy_pos)
        if bestLane is not None:
            if bestLane.getID() != id :
                print( id + "is different" )
            detectors.append(sumolib.sensors.inductive_loop.InductiveLoop("sens_"+id, bestLane.getID(), pos))
        else:
            print("there is None baseline")
        #detectors.append(sumolib.sensors.inductive_loop.InductiveLoop("sens_"+id, id, xy_pos))
        '''

sumolib.files.additional.write(DetName , detectors)
print("done")