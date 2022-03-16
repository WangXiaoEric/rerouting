#!/usr/bin/python
# -*- coding: utf-8 -*-
#數據處理
from __future__ import absolute_import
from __future__ import print_function
import os
import sys
import optparse
from optparse import OptionParser
import csv
import math
from random import sample
import pandas as pd
import numpy as np

import traceback

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import sumolib

def CalMinsAvgSpeedSingleFile(outputdir , data ,LoopTime = 60):
    """
    將平均速度數據壓縮為分鐘級
    :param outputdir:
    :param data:
    :param LoopTime:
    :return:
    """
    outputdir = outputdir 
    global out
    out = pd.DataFrame(columns=["interval","avgspeed"])
    speedfactor = 1#0.95
    #print(len(data))
    StartIdx = 0
    t = 0
    for idx in range(len(data)):
        #print(idx)
        if idx == 0:
            t =  int(data.iloc[idx]["time"]/LoopTime) # calculate t every mins
            StartIdx = 0 # for pandas csv 
            if t !=0 : #not start from  0 
                for i in range(t):
                    newrow = {"interval":i ,"avgspeed": speedfactor * 13.89}
                    out = out.append(newrow, ignore_index=True)
                #print(len(out))
                #print("------")

        if t != int(data.iloc[idx]["time"]/LoopTime): #arrive next interval
                EndIdx = idx
                num = len(data[StartIdx:EndIdx]["avgspeed"])
                if num == 0:
                    avg_speed = 0
                else :
                    avg_speed = data[StartIdx:EndIdx]["avgspeed"].sum()/num

                interval = t
                newrow = {"interval":interval ,"avgspeed": avg_speed }
                out = out.append(newrow, ignore_index=True)
                StartIdx = idx # StartIdx = EndIdx
                #print(len(out))
                #print("up----")
                if int(data.iloc[idx]["time"]/LoopTime) != t+1: #next interval not continous, compliment 0
                    #print(int(data.iloc[idx]["time"]/LoopTime) - t)
                    for i in range(1,int(data.iloc[idx]["time"]/LoopTime) - t):
                        if out.tail(1)["avgspeed"].iloc[0] < speedfactor * 13.89/2:
                            newrow = {"interval":interval + i ,"avgspeed": 0 }
                        else:
                            newrow = {"interval":interval + i ,"avgspeed": speedfactor * 13.89 }
                        out = out.append(newrow, ignore_index=True)
                        #print(len(out))
                        #print("down------")

                t = int(data.iloc[idx]["time"]/LoopTime) # update t
    #print(len(out))
    #leave loop
    # EndIdx = idx + 1 #last one need to plus 1 此语句会报 reference before defination错误
    EndIdx = len(data) + 1 #last one need to plus 1
    num = len(data[StartIdx:EndIdx]["avgspeed"])
    if num == 0:
        avg_speed = 0
    else :
        avg_speed = data[StartIdx:EndIdx]["avgspeed"].sum()/num

    interval = t

    newrow = {"interval":interval ,"avgspeed": avg_speed }
    out = out.append(newrow, ignore_index=True)

    #再做結尾處理
    # 讀取summary文件  TODO  補齊即可
    summary = open('txt_result/out0/summary.txt', 'r')
    sime_time = int(summary.read())
    # print(sime_time)
    append_num = int(sime_time/LoopTime) + 1 - out.shape[0]
    if append_num > 0:
        for i in range(append_num):
            newrow = {"interval": interval + i + 1, "avgspeed": speedfactor * 13.89}
            out = out.append(newrow, ignore_index=True)



    #out.to_csv(outputdir, index=False)
    return out

def CalMinsInstSpeedSingleFile(outputdir , data , avg_pd , LoopTime = 60):
    """
    將瞬時速度數據壓縮為分鐘級，並與平均速度合併
    :param outputdir:
    :param data:
    :param avg_pd:
    :param LoopTime:
    :return:
    """
    outputdir = outputdir 
    global out
    out = pd.DataFrame(columns=["interval","instspeed"])
    speedfactor = 1#0.95
    #avg_pd = avg_pd.drop(columns="interval")
    #print(len(data))
    StartIdx = 0
    t = 0
    for idx in range(len(data)):
        #print(idx)
        if idx == 0:
            t =  int(data.iloc[idx]["time"]/LoopTime) # calculate t every mins
            StartIdx = 0 # for pandas csv 
            if t !=0 : #not start from  0 
                for i in range(t):
                    newrow = {"interval":i ,"instspeed": 0}
                    out = out.append(newrow, ignore_index=True)
                #print(len(out))
                #print("------")

        if t != int(data.iloc[idx]["time"]/LoopTime): #arrive next interval
                EndIdx = idx
                num = len(data[StartIdx:EndIdx]["instspeed"])
                if num == 0:
                    avg_speed = 0
                else :
                    avg_speed = data[StartIdx:EndIdx]["instspeed"].sum()/num

                interval = t
                newrow = {"interval":interval ,"instspeed": avg_speed }
                out = out.append(newrow, ignore_index=True)
                StartIdx = idx # StartIdx = EndIdx
                #print(len(out))
                #print("up----")
                if int(data.iloc[idx]["time"]/LoopTime) != t+1: #next interval not continous, compliment 0
                    #print(int(data.iloc[idx]["time"]/LoopTime) - t)
                    for i in range(1,int(data.iloc[idx]["time"]/LoopTime) - t):
                        newrow = {"interval":interval + i ,"instspeed": 0 }
                        out = out.append(newrow, ignore_index=True)
                        #print(len(out))
                        #print("down------")

                t = int(data.iloc[idx]["time"]/LoopTime) # update t
    #print(len(out))
    #leave loop
    # EndIdx = idx + 1 #last one need to plus 1
    EndIdx = len(data) + 1
    num = len(data[StartIdx:EndIdx]["instspeed"])
    if num == 0:
        avg_speed = 0
    else :
        avg_speed = data[StartIdx:EndIdx]["instspeed"].sum()/num

    interval = t 

    newrow = {"interval":interval ,"instspeed": avg_speed }
    out = out.append(newrow, ignore_index=True)

    # 讀取summary文件  TODO  補齊即可
    summary = open('txt_result/out0/summary.txt', 'r')
    sime_time = int(summary.read())
    # print(sime_time)
    append_num = int(sime_time / LoopTime) + 1 - out.shape[0]
    if append_num > 0:
        for i in range(append_num):
            newrow = {"interval": interval + i + 1, "instspeed": speedfactor * 13.89}
            out = out.append(newrow, ignore_index=True)


    if len(out) >= len(avg_pd):
        end = out["interval"].tolist()[-1]
        for i in range(1,len(out) - len(avg_pd)+1):
            newrow = {"interval":end + i ,"avgspeed": 13.89 }
            avg_pd = avg_pd.append(newrow, ignore_index=True)
        out = pd.merge(out,avg_pd, on='interval')

    else:
        end = avg_pd["interval"].tolist()[-1]
        for i in range(1,len(avg_pd) - len(out)+1):
            newrow = {"interval":end + i ,"instspeed": 0 }
            out = out.append(newrow, ignore_index=True)
        out = pd.merge(out,avg_pd, on='interval')

    for i in range(len(out)):
        if out.loc[i,"avgspeed"] >= 11 and out.loc[i,"instspeed"] == 0:
            out.loc[i,"instspeed"] = 13.89

    out.to_csv(outputdir, index=False)
    return out

def GetRandom(t , VehNum,seq):
    if VehNum < 10:
        chosenum = 4
    else:
        chosenum = math.ceil(VehNum * 0.4)

    if len(seq) < chosenum:
        chosenum = len(seq)
        #print( str(300*t)+"~"+str( 300*(t+1) )+" < 4: "+str(chosenum) )
    seq = sample(seq,chosenum)
    return seq

def MakeSelectedVehicleData(interval,selectedlist,avginst_data,avgidx ):
    SelectedAvg_data = pd.DataFrame(columns=["interval","SelectedAvgspeed","avginstspeed"])
    for speed in selectedlist:
        newrow = {"interval":interval ,"SelectedAvgspeed": speed , "avginstspeed": avginst_data["avginstspeed"][avgidx]}
        SelectedAvg_data = SelectedAvg_data.append(newrow, ignore_index=True)
    return SelectedAvg_data

def CalZvalue(SelectedAvg_data):
    Z_data = pd.DataFrame(columns=["interval","Zvalue"])
    Z_data["interval"] = SelectedAvg_data["interval"]
    Z_data["Zvalue"] = SelectedAvg_data["SelectedAvgspeed"] / SelectedAvg_data["avginstspeed"] 
    return Z_data

def CalZSvalue(Z_data):
    ZS_data = pd.DataFrame(columns=["interval","ZSvalue"])
    Zmax = Z_data["Zvalue"].max() # 取得此區間的最大Z值
    Zmin = Z_data["Zvalue"].min()
    TotalNum = len( Z_data["Zvalue"])
    ZS_data["interval"] = Z_data["interval"]
    ZS_data["ZSvalue"] =  ( (Z_data["Zvalue"] - Zmin ) / ( Zmax - Zmin ) ) * ( (TotalNum - 1) / (TotalNum) ) + 1 / (2 * (TotalNum))
    return ZS_data

def SelectVehicleAndZ(outputdir, avg_data , inst_data ,LoopTime = 300):

        #outputdir = Dir + '/SelectSpeed_' + lane
        #if not os.path.exists(outputdir):
        #    os.makedirs(outputdir)
        '''
        inputdir = Dir + '/5minInstSpeed_' + lane
        '''

        #for outeridx in range(filenum):
        '''
        try:
            avginst_data = pd.read_csv(inputdir+"/"+lane+"_"+str(fileidx)+".csv")
            #print(inputdir+"/"+lane+"_"+str(outeridx)+".csv")
            data = pd.read_csv(Dir +'/Preprocess_' + lane+"/"+lane+"_"+str(fileidx)+".csv")
            #print(Dir +'/Preprocess_' + lane+"/"+lane+"_"+str(i)+".csv")
            
        except:
            print("Select File "+lane+"_"+str(fileidx)+".csv not exist")
            continue
        '''

        orig_avg_data = pd.DataFrame(columns=["avg-speed","End-exit"])
        orig_avg_data["avg-speed"] = avg_data["avgspeed"]
        orig_avg_data["End-exit"] = avg_data["time"]
        Final_data = pd.DataFrame(columns=["interval","Zmax","Zmin","q_value","SelectedNum"])

        avginst_data = pd.DataFrame(columns=["interval","avginstspeed"])
        avginst_data["interval"] =  inst_data["interval"]
        avginst_data["avginstspeed"] = inst_data["instspeed"]

        StartIdx = 0
        t = 0
        for idx in range(len(orig_avg_data)):
            if idx == 0:
                t = int(orig_avg_data.iloc[idx]["End-exit"]/LoopTime) # 以每五分鐘一個區間
                StartIdx = 0
                if t !=0 : #一開始不是從 0~5分鐘開始
                    for i in range(t):
                        newrow = {"interval":i ,"Zmax":1,"Zmin":0,"q_value":0,"SelectedNum":0}
                        Final_data = Final_data.append(newrow, ignore_index=True)
            
            if t != int(orig_avg_data.iloc[idx]["End-exit"]/LoopTime): # 下一個區間
                EndIdx = idx
                interval = t
                num = len(orig_avg_data[StartIdx:EndIdx]["avg-speed"])
                if interval >= len(avginst_data["interval"]):
                    break
                avgidx = int(avginst_data[avginst_data["interval"] == interval].index[0]) #取得avginst_data 中的 index
                if avginst_data["avginstspeed"][avgidx] == 0: #代表此時無車輛經過，就將此selectedvehicle刪除，否則計算會出錯，avg/avginst
                    newrow = {"interval":interval ,"Zmax":1,"Zmin":0,"q_value":0,"SelectedNum":0}
                    Final_data = Final_data.append(newrow, ignore_index=True)
                else:
                    # 隨機取得車輛
                    selectedlist = GetRandom(t , num ,orig_avg_data[StartIdx:EndIdx]["avg-speed"].tolist()) 
                    #製作Z值得data
                    SelectedAvg_data = MakeSelectedVehicleData(interval,selectedlist,avginst_data,avgidx )
                    #計算 Z 值
                    Z_data = CalZvalue(SelectedAvg_data)
                    Zmax = Z_data["Zvalue"].max() # 取得此區間的最大Z值
                    Zmin = Z_data["Zvalue"].min()# 取得此區間的最小Z值
                    TotalNum = len( Z_data["Zvalue"])#取得選擇的車輛數
                    # 計算 Z* 值
                    ZS_data = CalZSvalue(Z_data)
                    # 計算 q 值
                    total = np.log(ZS_data["ZSvalue"].astype(float)).sum()
                    q_value = -1 * total / (TotalNum)
                    newrow = {"interval":interval ,"Zmax":Zmax,"Zmin":Zmin,"q_value":q_value,"SelectedNum":TotalNum}
                    Final_data = Final_data.append(newrow, ignore_index=True)
                StartIdx = EndIdx
                if int(orig_avg_data.iloc[idx]["End-exit"]/LoopTime) != t+1: # 如果不是連續的區間，就補0
                    for i in range(1,int(orig_avg_data.iloc[idx]["End-exit"]/LoopTime) - t):
                        newrow = {"interval":interval + i ,"Zmax":1,"Zmin":0,"q_value":0,"SelectedNum":0}
                        Final_data = Final_data.append(newrow, ignore_index=True)
                t = int(orig_avg_data.iloc[idx]["End-exit"]/LoopTime)

        #最後一個 要加1
        # EndIdx = idx + 1
        EndIdx = len(data) + 1
        num = len(orig_avg_data[StartIdx:EndIdx]["avg-speed"])
        interval = t 

        if interval >= len(avginst_data["interval"]):
            pass
        else:
            avgidx = int(avginst_data[avginst_data["interval"] == interval].index[0]) #取得avginst_data 中的 index
            if avginst_data["avginstspeed"][avgidx] == 0: #代表此時無車輛經過，就將此selectedvehicle刪除，否則計算會出錯，avg/avginst
                newrow = {"interval":interval ,"Zmax":1,"Zmin":0,"q_value":0,"SelectedNum":0}
                Final_data = Final_data.append(newrow, ignore_index=True)
            
            else:
                # 隨機取得車輛
                selectedlist = GetRandom(t , num ,orig_avg_data[StartIdx:EndIdx]["avg-speed"].tolist()) 
                #製作Z值得data
                SelectedAvg_data = MakeSelectedVehicleData(interval,selectedlist,avginst_data,avgidx )
                #計算 Z 值
                Z_data = CalZvalue(SelectedAvg_data)
                Zmax = Z_data["Zvalue"].max() # 取得此區間的最大Z值
                Zmin = Z_data["Zvalue"].min()# 取得此區間的最小Z值
                TotalNum = len( Z_data["Zvalue"])#取得選擇的車輛數
                # 計算 Z* 值 formalization后的数值
                ZS_data = CalZSvalue(Z_data)
                # 計算 q 值 指数方求和
                total = np.log(ZS_data["ZSvalue"].astype(float)).sum()
                q_value = -1 * total / (TotalNum)
                newrow = {"interval":interval ,"Zmax":Zmax,"Zmin":Zmin,"q_value":q_value,"SelectedNum":TotalNum}
                Final_data = Final_data.append(newrow, ignore_index=True)

        # 讀取summary文件  TODO  補齊即可
        summary = open('txt_result/out0/summary.txt', 'r')
        sime_time = int(summary.read())
        # print(sime_time)
        append_num = int(sime_time / LoopTime) + 1 - Final_data.shape[0]
        if append_num > 0:
            for i in range(append_num):
                newrow = {"interval":interval + i + 1 ,"Zmax":1,"Zmin":0,"q_value":0,"SelectedNum":0}
                Final_data = Final_data.append(newrow, ignore_index=True)

        Final_data.to_csv(outputdir, index=False)

if __name__ == "__main__":
    parser = OptionParser()#parser = OptionParser()
    parser.add_option("-s", "--start", dest="start",type="int" , default="0", help="The start used to run SUMO start from csv_result/outx")
    parser.add_option("-e", "--end", dest="end", type="int" ,default="1", help="The end used to run SUMO end at csv_result/outx")
    parser.add_option("-i", "--isdebug", dest="isdebug", type="int" ,default="0", help="1 is debug - interavg_result/out' + lstm;  0 is common case using interavg_result/out")

    (options, args) = parser.parse_args()
    start = options.start
    end = options.end
    isdebug = options.isdebug
    if isdebug == 1:
        lstm = "lstm"
    else:
        lstm = ""

    NetName = "data/01/Tainan.net.xml"
    net = sumolib.net.readNet(NetName)
    edges = net.getEdges()
    Lane_list = []

    for edge in edges:
        lanes = edge.getLanes()
        for lane in lanes:
            Lane_list.append(lane.getID())

    # Lane_list.append("24452786#0_0")
    #Lane_dict[] = 5
    speedFactor = 1.5
    for i in range(start,end):
        print("i= ", i)
        if not os.path.exists('interavg_result/out'  + lstm + str(i)):
            os.makedirs('interavg_result/out' + lstm + str(i))
        if not os.path.exists('interZ_result/out' + lstm + str(i)):
            os.makedirs('interZ_result/out' + lstm + str(i))

        for lane in Lane_list:
            try:
                data = pd.read_csv('csv_result/out' + lstm+str(i)+"/"+lane+".csv")
                #except:
                #    continue
                tempavg = pd.DataFrame(columns=["time","avgspeed"])
                tempavg["avgspeed"] = data["length"] / (data["exit"] - data["entry"]).abs()
                tempavg["time"] = data["exit"]
                tempavg = tempavg[~tempavg.isin([np.nan, np.inf, -np.inf]).any(1)]
                a = np.array(tempavg["avgspeed"].values.tolist())

                tempavg["avgspeed"] = np.where(a > 13.89*speedFactor, 13.89*speedFactor, a).tolist()

                tempinst = pd.DataFrame(columns=["time","instspeed"])
                tempinst["time"] = data["mid"]
                tempinst["instspeed"] = data["instspeed"]
                #原先数据为按照exit排序，需要重新按照Mid排序
                tempinst = tempinst.sort_values(by=["time"])

                #tempavg.to_csv("123.csv" , index=False)  以五分鐘為間隔，統計平均速度
                avg_pd = CalMinsAvgSpeedSingleFile('interavg_result/out' + lstm+str(i)+"/"+lane +".csv" , tempavg ,300)
                # 以五分鐘為間隔，統計平均速度，並且合併速度
                speed_pd = CalMinsInstSpeedSingleFile('interavg_result/out' + lstm+str(i)+"/"+lane +".csv" , tempinst , avg_pd ,300)
                inst_pd = pd.DataFrame(columns=["time","instspeed"])
                inst_pd['interval'] = speed_pd['interval']
                inst_pd['instspeed'] = speed_pd['instspeed']
                SelectVehicleAndZ('interZ_result/out' + lstm+str(i)+"/SelectSpeed_"+lane +".csv" , tempavg , inst_pd ,LoopTime = 300)
                
            except Exception as e:
                if isinstance(e, FileNotFoundError):
                    print(e)
                else:
                    traceback.print_exc()
                continue


