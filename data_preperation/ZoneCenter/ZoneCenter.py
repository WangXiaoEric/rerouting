from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import math
import pandas as pd
sys.path.append(os.path.join(os.environ["SUMO_HOME"], 'tools'))
import sumolib
from sklearn.cluster import KMeans
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from collections import Counter


def sortXY(X, Y, levelnum): #levelnum: 一層有多少point
  final_data = []
  cord_dict = {}
  for idx , x in enumerate(X):
    cord_dict[x] = idx
  for i in range( math.ceil(len(Y)/levelnum) ):
    X_data = [] # for sort
    
    if i == 3:
        for j in range(2):
            idx = Y.index( max(Y) )# 取出max y 的 index
            #移除此座標
            Y.pop(idx)
            X_data.append(X[idx])
            X.pop(idx)
            if not Y:
               break
    else:
        
        for j in range(levelnum):
            idx = Y.index( max(Y) )# 取出max y 的 index
            #移除此座標
            Y.pop(idx)
            X_data.append(X[idx]) 
            X.pop(idx)
            if not Y:
                break
    
    X_data.sort()
    for x in X_data:
      final_data.append(cord_dict[x])
    new_order = np.array(final_data)
    
  return new_order

def rearrange_labels(AllPos, cluster_labels, levelnum=3):
    labels, ctrs = [], []
    x = []
    y = []
    for i in range(len(set(cluster_labels))):
        AllPosi = AllPos[cluster_labels == i]
        ctr = np.mean(AllPosi, axis=0)
        labels.append(i)
        ctrs.append(ctr)
        x.append(ctr.tolist()[0])
        y.append(ctr.tolist()[1])

    ctrs = np.row_stack(ctrs)
    labels = np.array(labels).reshape(-1, 1)
    # sort on x column
    #new_order = ctrs[:, sort_on_column].argsort()                    
    new_order = sortXY(x, y, levelnum)
    labels_new = []
    for num in labels[new_order]:
      labels_new.append(num[0])

    ctrs_new = ctrs[new_order]

    for idx in range(len(cluster_labels)):
      cluster_labels[idx] = labels_new.index(cluster_labels[idx])

    return cluster_labels, ctrs_new

def GetCutEdgeName(EdgeName):
    if EdgeName  != "":
        if "街" in EdgeName:
            section = EdgeName.split("街")
            return section[0]
        elif "路" in EdgeName:
            section = EdgeName.split("路")
            return section[0]
        else:
            return EdgeName
    else:
        return "No name"



## Start 

classnum = 14 ## total class
levelnum = 3 ## there is 3 zone in every level
offset = 1
NetXml = "Tainan.net.xml"

net = sumolib.net.readNet(NetXml)
edges = net.getEdges()


AllPos = [] # [ [x1,y1],[x2,y2],....... ]
## get the middle coordinate in the lane
for edge in edges:

    lanes = edge.getLanes()
    for lane in lanes:
        posx = 0
        posy = 0
        for pos in lane.getShape():
            posx += pos[0]
            posy += pos[1]
        x =  posx / len(lane.getShape()) 
        y =  posy / len(lane.getShape())
        AllPos.append([x,y])
        

AllPos = np.array(AllPos)
#請KMeans分成三類
clf = KMeans(n_clusters=classnum,algorithm="auto")

#開始訓練！
clf.fit(AllPos)
## rerange the label 
cluster_labels, ctrs = rearrange_labels(AllPos=AllPos, cluster_labels=clf.labels_ , levelnum = levelnum)


#最後畫出來看看#
#plt.scatter(AllPos[:,0],AllPos[:,1], c=,s=classnum)#clf.labels_
fig, ax = plt.subplots()
for i, m in enumerate(ctrs):
    print(m[[0, 1]])
    ax.annotate(
        xy=m[[0, 1]],
        text=i+offset,#
        bbox=dict(boxstyle="square", fc="w", ec="grey", alpha=0.9),
    )
ax.scatter(AllPos[:, 0], AllPos[:, 1], c=cluster_labels)
plt.savefig("classification_test.png") 
