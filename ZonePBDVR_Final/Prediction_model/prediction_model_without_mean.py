# -*- coding: utf-8 -*-
import numpy as np
import os
import traci
import math
import pandas as pd
from pathlib import Path
import traceback

"""the speed prediction model: estimate the predicted speed for target RS"""
#def getSpeed(RS, RS_Kjam, speed_result, vehicle, footprint, current_time, depart_time):
def getSpeed(RS, five_minu_loopd_avg_speed_result, current_time, depart_time, model, predicted_speed, MeanSpeed_deprecated, MeanZ, Bd):
    """
    five_minu_loopd_avg_speed_result 基本思路：从这里面获取12个，然后再预测出一个。 如果没有12个，则从文件数据中倒退进行获取
    :param RS:
    :param five_minu_loopd_avg_speed_result:
    :param current_time:
    :param depart_time:
    :param model:
    :param predicted_speed: 已經LSTM預測過的緩存速度
    :param MeanSpeed:
    :param MeanZ:
    :param Bd:
    :return:
    """
    """<--comment"""
    #  平均速度 =  (歷史瞬時速度 + LSTM誤差)-> 瞬時轉平均
    # five_minu_loopd_avg_speed_result: 儲存SUMO從開始到結束，每5分鐘的瞬時速度
    # predicted_speed: 將LSTM已經預測的誤差緩存起來  - 注意这点略有所不同
    # pred_num: 當前車輛需使用LSTM預測次數
    # pred_len: 目前RS已經緩存的誤差數量
    # maxspeed(道路速限): 13.89(m/s) == 50(km/h) SUMO的速度單位是(m/s)
    """comment-->"""
    # 如果当前路段没有训练出的Model 则反馈最大速度
    try:
        if model == None or model == 0:
            pred = 13.89
            return pred, predicted_speed

        #间隔的次数，需要预测pred_numth这次的速度，那么需要循环预测
        pred_numth = int((depart_time - current_time)/300) + 1

        #CASE1 缓存如果有，则直接从缓存中获取
        if len(predicted_speed[RS]) >= pred_numth:
            # try:
            predicted_speed[RS][pred_numth - 1]
            # except Exception as e:
            #     traceback.print_exc()
            return predicted_speed[RS][pred_numth-1], predicted_speed
        # predicted_speed[RS].append(pred_inst[0][0].tolist())

        #CASE2 缓存中没有则逐步预测

        # 先将过去每隔五次间隔数据放进来
        historical_lstm_input_data = five_minu_loopd_avg_speed_result[RS][:]
        #将已经预测过的缓存数据放进来
        historical_lstm_input_data.extend(predicted_speed[RS][:])
        #循环迭代预测
        for i in range(pred_numth - len(predicted_speed[RS])):
            #往往第一次才走这种缺乏数据的情况
            if len(historical_lstm_input_data) < 12:
                #先从five_minu_loopd_avg_speed_result将数据全部取过来
                # historical_lstm_input_data = five_minu_loopd_avg_speed_result[RS][-12 + i:]
                #TODO 添加判断，如果文件不存在则从冷启动数据中获取数据 interavg_result/out0_init/
                avginst_data = None
                if os.path.exists('interavg_result/out0' + RS + '_0.csv'):
                    avginst_data = pd.read_csv("interavg_result/out0" + "/" + RS +"_0.csv")
                elif Path("interavg_result/out0_init/" + RS + "_0.csv").exists():
                    avginst_data = pd.read_csv("interavg_result/out0_init/" + RS + "_0.csv")
                #差多少数据，补充多少数据，凑够12条数据
                required_data = avginst_data.loc[:, 'instspeed'][-(12 - len(historical_lstm_input_data)):].values
                # historical_lstm_input_data = required_data.append(historical_lstm_input_data)
                historical_lstm_input_data = np.append(required_data, historical_lstm_input_data)

                X = [[[0] * 1 for i in range(12)]]
                for i in range(12):
                    X[0][i][0] = historical_lstm_input_data[i]

                # try:
                pred_inst = model.predict(np.array(X))
                # except Exception as e:
                #     traceback.print_exc()

                pred = pred_inst[0][0]
                if pred <= 0:
                    pred = 0.01
                if pred > 15:
                    pred = 15

                #将预测好的数据添加进来
                historical_lstm_input_data = historical_lstm_input_data.tolist()
                historical_lstm_input_data.append(pred)
                predicted_speed[RS].append(pred)
            else:
                historical_lstm_input_data = historical_lstm_input_data[-12:]

                X = [[[0] * 1 for i in range(12)]]
                for i in range(12):
                    X[0][i][0] = historical_lstm_input_data[i]

                # try:
                pred_inst = model.predict(np.array(X))
                # except Exception as e:
                #     traceback.print_exc()

                pred = pred_inst[0][0]
                if pred <= 0:
                    pred = 0.01
                # """<--comment"""
                # # speedFactor="normc(0.95,0.10,0.60,1.50)
                # # SUMO官網定義 vehicle speed = min( maxSpeed(官網:最高車速55.55(m/s)), speedFactor * speedLimit(速限13.89，我們定義))
                # # 所以整個路網 最低車速 = 13.89 x 0.6 。最高車速 = 13.89 x 1.5
                # # 但是車輛難以在此路網達到最高速，故以15回傳
                # """comment-->"""
                if pred > 15:
                    pred = 15

                historical_lstm_input_data.append(pred)
                predicted_speed[RS].append(pred)
            #增加了一个数据，要删除一个数据, 即将第一个数据去除
            historical_lstm_input_data = historical_lstm_input_data[-12:]

        #找到当前MeanZ的位置
        lastidx = len(five_minu_loopd_avg_speed_result[RS]) + pred_numth
        if MeanZ == None or len(MeanZ["Zmax"]) <= lastidx:
            pred = historical_lstm_input_data[-1]
        else:
            pred = historical_lstm_input_data[-1] * ((MeanZ["Zmax"][lastidx] - MeanZ["Zmin"][lastidx]) * math.pow(Bd, MeanZ["q_value"][lastidx]) +
                    MeanZ["Zmin"][lastidx])


        return pred ,predicted_speed

    except Exception as e:
        traceback.print_exc()
        pred = 13.89
        return pred, predicted_speed


"""the TL prediction model: estimate the queuing time before leaving the target RS"""


def getQueuingTime(arriving_time, RS, RS_Length, TL, phase_list):
    leaveTime_of_one_veh = 2.5  # 每輛駛離時間(單位:s)

    # TL_id = TL[0]
    index = int(TL[1])
    Tc = 0  # cycle
    Tp = 0  # green phase + yellow phase
    state_cycle = ""  # RS 的 phase 週期變化
    Ts = 0  # timing plan StartTime (first green phase start time)
    """estimate Tc and Tp"""
    for phase in phase_list:

        Tc = Tc + int(phase[0])
        if phase[1][index] == "G" or phase[1][index] == "g" or phase[1][index] == "y":
            Tp = Tp + int(phase[0])
        state_cycle = state_cycle + phase[1][index]

    # 週期裡沒有red phase ex:"GyGy"
    if state_cycle.find("r") == -1:
        return 0

    """estimate Ts"""
    # 時制開始必為綠燈 or紅燈，故不考慮黃燈
    first_green_phase_pos = min(state_cycle.find("G"), state_cycle.find("g"))
    # 只有 "G" or "g"
    if first_green_phase_pos == -1:
        first_green_phase_pos = max(state_cycle.find("G"), state_cycle.find("g"))
    if first_green_phase_pos == 0:
        Ts = 0
    else:
        # 把 first green phase 前 phase 的 duration 加總
        for i in range(first_green_phase_pos):
            Ts = Ts + int(phase_list[i][0])

    """estimate queuing_time"""
    remain_red_phase = 0
    accumulate_veh = 0
    # 車輛到達TL時間 < 時制開始時間 (紅燈)
    if arriving_time < Ts:
        remain_red_phase = Ts - arriving_time
        elapsed_red_phase = (Tc - Tp) - remain_red_phase
        # if RS not in RS_acc:
        # 假設每秒累積車輛數 = 0.15
        accumulate_veh = elapsed_red_phase * 0.15
        # 最大排隊數:RS_Length/7.5
        if accumulate_veh > RS_Length / 7.5:
            accumulate_veh = RS_Length / 7.5
            # else:
        #    accumulate_veh = estimateAccumulateVeh(RS, elapsed_red_phase)

        RS_queuing_time = remain_red_phase + (accumulate_veh * leaveTime_of_one_veh)

    else:
        # arrival_time 在當前周期的第幾秒
        time_in_this_period = (arriving_time - Ts) % Tc
        # red phase
        if time_in_this_period > Tp:
            remain_red_phase = Tc - time_in_this_period
            elapsed_red_phase = (Tc - Tp) - remain_red_phase
            # if RS not in RS_acc:
            accumulate_veh = elapsed_red_phase * 0.15
            if accumulate_veh > RS_Length / 7.5:
                accumulate_veh = RS_Length / 7.5
                # else:
            #    accumulate_veh = estimateAccumulateVeh(RS, elapsed_red_phase)

            RS_queuing_time = remain_red_phase + (accumulate_veh * leaveTime_of_one_veh)
        # green phase
        else:
            RS_queuing_time = 0

    # print("remain_red_phase:",remain_red_phase)
    # print("accumulate_veh:",accumulate_veh)
    return RS_queuing_time


"""estimate the average number of vehicles accumulated in the queue """


def estimateAccumulateVeh(RS, elapsed_red_phase):
    time_interval = 5  # 每�?秒�?算�?次累積�?輛數
    idx = int(elapsed_red_phase / time_interval)
    x = elapsed_red_phase % time_interval

    if idx >= len(RS_acc[RS]) - 1:
        y1 = RS_acc[RS][-1]
        y2 = RS_acc[RS][-1]
    else:
        y1 = RS_acc[RS][idx]
        y2 = RS_acc[RS][idx + 1]

    y = (y2 - y1) / time_interval * x + y1

    return y
