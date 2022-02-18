from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import math
sys.path.append(os.path.join(os.environ["SUMO_HOME"], 'tools'))
import sumolib
def ComputeDistance(FromPos , SelectedPos):
    x = FromPos[0] - SelectedPos[0]
    y = FromPos[1] - SelectedPos[1]
    sum = x*x + y*y
    distance = math.sqrt(sum)
    return distance
def GetStartPoint(coord):
    mindis = 999999999
    minidx = -1
    StartPos = coord[0]
    end = len(coord) - 1 # avoid final position (Node)
    for idx in range( 1, end): # avoid initial position (Node)
        SelectedPos = coord[idx]
        dis = ComputeDistance(StartPos , SelectedPos)
        if dis < mindis :
            mindis = dis
            minidx = idx
    if minidx == -1 :
        print("minidx error")
        exit()
    return minidx
def GetEndPoint(coord):
    mindis = 999999999
    minidx = -1
    EndPos = coord[len(coord) - 1]
    end = len(coord) - 1 # avoid final position (Node)
    for idx in range( 1, end): # avoid initial position (Node)
        SelectedPos = coord[idx]
        dis = ComputeDistance(EndPos , SelectedPos)
        if dis < mindis :
            mindis = dis
            minidx = idx
    if minidx == -1 :
        print("minidx error")
        exit()
    return minidx

net = sumolib.net.readNet("Tainan.net.xml")
edges = net.getEdges()

wf = open("SensCoord.txt", mode='w')
i=0

for edge in edges:
    lanes = edge.getLanes()
    for lane in lanes:
        #if len(lane.getShape()) %2 == 1:
        '''if lane.getID() == "161010072#3_0" or lane.getID() == "-161010072#5_0":
            print(lane.getID())
            print("false")
            print(lane.getShape())
            print("True")
            print()
            
            print("start: ")
            print(lane.getShape(includeJunctions=True)[startidx])
            endidx = GetEndPoint(lane.getShape(includeJunctions=True))
            print("end: ")
            print(lane.getShape(includeJunctions=True)[endidx])
            print("---------------------------------")
        '''
        line = lane.getID()
        # Start Pos
        laneshape = lane.getShape(includeJunctions=True)
        startidx = GetStartPoint(laneshape)
        line += "," + str( laneshape[startidx][0] ) + "," + str( laneshape[startidx][1] )
        
        # Mid Pos 
        posx = 0
        posy = 0
        for pos in lane.getShape():
            posx += pos[0]
            posy += pos[1]
        line += "," + str( posx / len(lane.getShape()) ) + "," + str( posy / len(lane.getShape()) )

        # End Pos
        endidx = GetEndPoint(laneshape)
        line += "," + str( laneshape[endidx][0] ) + "," + str( laneshape[endidx][1] )
        
        if i==0:
            wf.write(line)
            i+=1
        else:
            wf.write("\n"+line)
        
wf.close()

'''
nodes = net.getNodes()
wf = open("NodeConn.txt", mode='w')
i=0
for node in nodes:
    neighbors = node.getNeighboringNodes()
    line = node.getID()
    for neighbor in neighbors:
        line += "," + neighbor.getID()
    if i==0:
        wf.write(line)
        i+=1
    else:
        wf.write("\n"+line)
wf.close()
'''
'''
for node in nodes:
    if node.getID() == "296967344": 
        print(node.getCoord())
        neighbors = node.getNeighboringNodes()
        for neighbor in neighbors:
              print(neighbor.getID())
'''