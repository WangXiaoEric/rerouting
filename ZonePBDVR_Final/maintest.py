#COST2與COST1的不同
# 1、 COST1只用當前當下速度  COST2是可以用LSTM，再把瞬時速度轉為每一輛車的平均速度
# 2、 COST1沒有DB  COST2有DB
# 3、 COST1沒有用LSTM COST2用到LSTM（並且是預測的是error）


# Attention:必須把Cost1 run完畢（睦昆建議可以先跑10次），才能run2.
# COST2run完，會產生檔案，然後用新檔案再run一次COst2

from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
from optparse import OptionParser
#import random
import time
#import networkx as nx
import numpy as np
#from numpy import ndarray
import gc
import get_network_info as gni
import traffic_congestion_prediction as tcp
import vehicle_selection as vs
import vehicle_rank as vr
import reroute_algorithm as ra
import xml.etree.cElementTree as ET
#from tensorflow.python.keras.models import load_model
#import json

#讀取summary文件
X = [i for i in range(8)]

p = X[-12:]




print(X)
print(p)