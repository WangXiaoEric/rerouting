# -*- coding: utf-8 -*-
import numpy as np
import traci
import math

"""the speed prediction model: estimate the predicted speed for target RS"""
#def getSpeed(RS, RS_Kjam, speed_result, vehicle, footprint, current_time, depart_time):
def getSpeed(RS, five_minu_loopd_avg_speed_result, current_time, depart_time, model, predicted_speed, MeanSpeed, MeanZ, Bd):
    """

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

    # __5___10----
    if model == 0:
        pred = 13.89
        return pred , predicted_speed
        
    PastTime = 3
    pred_num = int((depart_time - current_time)/300) + 1
    temp_pastspeed_list = five_minu_loopd_avg_speed_result[RS]
    pred_len = len(predicted_speed[RS])
    maxspeed = 13.89

    #if len(temp_pastspeed_list) % 5 !=0:
    #    print("temp_pastspeed_list is changed")
    #    exit(0)

    """<--comment"""
    # predtime 可刪除
    predtime = 32
    # Pred_limit 最多預測6次
    Pred_limit = 6
    """comment-->"""

    '''
    if pred_len >= len(MeanSpeed) - 3 and  pred_num >=pred_len:
        pred_inst = predicted_speed[RS][pred_len-1]
        return pred_inst , predicted_speed
    '''
       
    #pred_len 緩存已經LSTM預測過的數量
    """<--comment"""
    # 緩存中已經有需要的誤差，進入if
    """comment-->"""
    if pred_len >= pred_num: #Not need to predict again
        try:
            pred_inst = predicted_speed[RS][pred_num-1]
            lastidx = len(temp_pastspeed_list) + pred_num -1
            """<--comment"""
            # 此道路沒有歷史瞬時速度(沒有車子通過) pred = 速限 + 預測的誤差
            """comment-->"""
            if len(MeanSpeed) == 0:
                pred = pred_inst + maxspeed
            else:
                """<--comment"""
                # 超出所儲存的歷史瞬時速度的數量，pred = 速限速度
                """comment-->"""
                if lastidx > len(MeanSpeed) - 1: # over list length
                    pred = maxspeed 
                else:
                    """<--comment"""
                    # 超出所儲存的Zmax、Zmin的數量，pred = 速限速度
                    """comment-->"""
                    if len(MeanZ) == 0  or lastidx > len(MeanZ) - 1 or MeanZ["Zmax"][lastidx] == 0:
                        pred = maxspeed 
                    else:
                        pred = pred_inst + MeanSpeed[lastidx]
                        pred = pred *  ( (MeanZ["Zmax"][lastidx] - MeanZ["Zmin"][lastidx]) * math.pow(Bd, MeanZ["q_value"][lastidx]) + MeanZ["Zmin"][lastidx])

            if pred <= 0:
                pred = 0.01

            """<--comment"""
            # speedFactor="normc(0.95,0.10,0.60,1.50)
            # SUMO官網定義 vehicle speed = min( maxSpeed(官網:最高車速55.55(m/s)), speedFactor * speedLimit(速限13.89，我們定義))
            # 所以整個路網 最低車速 = 13.89 x 0.6 。最高車速 = 13.89 x 1.5
            # 但是車輛難以在此路網達到最高速，故以15回傳
            """comment-->"""
            if pred > 15:
                pred = 15
        except:
            print("list out of index")
            exit(0)
        return pred , predicted_speed
    else:
        """<--comment"""
        # 緩存中沒有需要的誤差，進入else
        # error: pred_num - pred_len: 緩存少幾筆誤差
        """comment-->"""
        error = pred_num - pred_len
        X = [[[0]*1 for i in range(PastTime)]]
        past_speed_list = temp_pastspeed_list + predicted_speed[RS]
        """<--comment"""
        # 當len(past_speed_list) < 3 。表示SUMO處於模擬剛開始的15分鐘以內。進入if
        """comment-->"""
        if len(past_speed_list) < PastTime :
            new_idx = PastTime-len(past_speed_list)
            for idx,i in enumerate(past_speed_list):
                if len(MeanSpeed) == 0:
                    X[0][new_idx][0] = i - maxspeed 
                else:
                    X[0][new_idx][0] = i - MeanSpeed[idx]
                    #print(MeanSpeed[idx])
                    new_idx +=1

                lastidx = idx
                
        else:
            """<--comment"""
            # 當len(past_speed_list) >= 3 。表示SUMO處於模擬開始的15分鐘以後。進入else
            """comment-->"""
            for idx,i in enumerate(past_speed_list[-PastTime:]):
                past_idx = past_speed_list.index(i)
                if len(MeanSpeed) == 0:
                    error_speed = past_speed_list[past_idx] - maxspeed 
                else:
                    """<--comment"""
                    # 超過已儲存的歷史瞬時速度，減去maxspeed
                    "       ""comment-->"""
                    if past_idx > len(MeanSpeed) - 1: # over list length
                       error_speed = past_speed_list[past_idx] - maxspeed 
                    else:
                        error_speed = past_speed_list[past_idx] - MeanSpeed[past_idx]

                X[0][idx][0] = error_speed
                

                lastidx = past_idx
        """<--comment"""
        # 預測超過30分鐘，直接帶入第30分鐘的歷史瞬時速度，以節省LSTM預測的時間
        """comment-->"""
        if pred_num > Pred_limit :
            lastidx += error
            if lastidx < len(MeanSpeed):
                if len(MeanZ) == 0 or lastidx > len(MeanZ) - 1 or MeanZ["Zmax"][lastidx] == 0 :
                    avgspeed = maxspeed 
                else:
                    avgspeed = MeanSpeed[lastidx] *  ( (MeanZ["Zmax"][lastidx] - MeanZ["Zmin"][lastidx]) * math.pow(Bd, MeanZ["q_value"][lastidx]) + MeanZ["Zmin"][lastidx])
                pred = avgspeed
                if pred <= 0:
                    pred = 0.01

                if pred > 15:
                    pred = 15

                return pred,predicted_speed
            else:
                return maxspeed  ,predicted_speed
        """<--comment"""
        # 開始預測誤差
        """comment-->"""
        for i in range(error):
            #print("enter prediction")
            pred_inst = model.predict(np.array(X))
            lastidx += 1
            # predict next , so move data forward
            for j in range(PastTime-1):
              X[0][j][0] = X[0][j+1][0]
            
            X[0][PastTime-1][0] = pred_inst[0][0].tolist()
            if len(MeanSpeed) == 0:
                pred = pred_inst[0][0].tolist() + maxspeed
            else:
                if lastidx > len(MeanSpeed) - 1: # over list length
                    pred = maxspeed 
                else:
                    if len(MeanZ) == 0  or lastidx > len(MeanZ) - 1 or MeanZ["Zmax"][lastidx] == 0:
                        pred = maxspeed 
                    else:
                        pred = pred_inst[0][0].tolist() + MeanSpeed[lastidx]
                        pred = pred *  ( (MeanZ["Zmax"][lastidx] - MeanZ["Zmin"][lastidx]) * math.pow(Bd, MeanZ["q_value"][lastidx]) + MeanZ["Zmin"][lastidx])
            
            
            predicted_speed[RS].append( pred_inst[0][0].tolist() )

            """<--comment"""
            # predtime 移除
            # if lastidx >= predtime:
            #    break
            """comment-->"""

        if pred <= 0:
            pred = 0.01

        if pred > 15:
            pred = 15

    return pred ,predicted_speed


"""the TL prediction model: estimate the queuing time before leaving the target RS"""
def getQueuingTime(arriving_time, RS, RS_Length, TL, phase_list):
    
    leaveTime_of_one_veh = 2.5 # 每�?駛離?��?(?��?:s)
        
    #TL_id = TL[0]
    index = int(TL[1])
    Tc = 0 # cycle
    Tp = 0 # green phase + yellow phase
    state_cycle = "" # RS ??phase ?��?變�?
    Ts = 0 # timing plan StartTime (first green phase start time)
    """estimate Tc and Tp"""
    for phase in phase_list:
        
        Tc = Tc + int(phase[0])
        if phase[1][index] == "G" or phase[1][index] == "g" or phase[1][index] == "y":
            Tp = Tp + int(phase[0])
        state_cycle = state_cycle + phase[1][index]
    
    # ?��?裡�??�red phase ex:"GyGy"
    if state_cycle.find("r") == -1:
        return 0
        
    """estimate Ts"""
    # ?�制?��?必為綠�? or紅�?，�?不考慮黃�?
    first_green_phase_pos = min(state_cycle.find("G"),state_cycle.find("g"))
    # ?��? "G" or "g"
    if first_green_phase_pos == -1:
        first_green_phase_pos = max(state_cycle.find("G"),state_cycle.find("g"))
    if first_green_phase_pos == 0:
        Ts = 0
    else:        
        # ??first green phase ??phase ??duration ?�總
        for i in range(first_green_phase_pos):
            Ts = Ts + int(phase_list[i][0])
            
    """estimate queuing_time"""
    remain_red_phase = 0
    accumulate_veh = 0
    # 車�??��?TL?��? < ?�制?��??��? (紅�?)
    if arriving_time < Ts:
        remain_red_phase = Ts - arriving_time
        elapsed_red_phase = (Tc - Tp) - remain_red_phase
        if RS not in RS_acc:
            # ?�設每�?累�?車�???= 0.15
            accumulate_veh = elapsed_red_phase * 0.15
            # ?�大�??�數:RS_Length/7.5
            if accumulate_veh > RS_Length/7.5:
                accumulate_veh = RS_Length/7.5                
        else:
            accumulate_veh = estimateAccumulateVeh(RS, elapsed_red_phase)
            
        RS_queuing_time = remain_red_phase + (accumulate_veh * leaveTime_of_one_veh)
               
    else:
        # arrival_time ?�當?�周?��?第幾�?
        time_in_this_period = (arriving_time - Ts) % Tc
        #red phase
        if time_in_this_period > Tp:
            remain_red_phase = Tc - time_in_this_period
            elapsed_red_phase = (Tc - Tp) - remain_red_phase
            if RS not in RS_acc:               
                accumulate_veh = elapsed_red_phase * 0.15
                if accumulate_veh > RS_Length/7.5:
                    accumulate_veh = RS_Length/7.5                
            else:
                accumulate_veh = estimateAccumulateVeh(RS, elapsed_red_phase)
                
            RS_queuing_time = remain_red_phase + (accumulate_veh * leaveTime_of_one_veh)
        #green phase
        else:
            RS_queuing_time = 0
            
    #print("remain_red_phase:",remain_red_phase)
    #print("accumulate_veh:",accumulate_veh)
    return RS_queuing_time

"""estimate the average number of vehicles accumulated in the queue """
def estimateAccumulateVeh(RS, elapsed_red_phase):
    
    time_interval = 5 # 每�?秒�?算�?次累積�?輛數
    idx = int(elapsed_red_phase/time_interval)
    x = elapsed_red_phase % time_interval
    
    
    if idx >= len(RS_acc[RS])-1:
        y1 = RS_acc[RS][-1]
        y2 = RS_acc[RS][-1]
    else:    
        y1 = RS_acc[RS][idx]
        y2 = RS_acc[RS][idx+1]
        
    y = (y2-y1)/time_interval*x + y1
    
   
    return y




