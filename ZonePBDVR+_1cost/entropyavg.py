#Deprecated.
import os
import sys
import pandas as pd
#Deprecated.
def GetMinMax(data_len):
    return max(data_len) , min(data_len)

def MakePdAndDict(data_max,data_min,columns):
    Index_list = []
    Num_dict = {}
    
    for i in range(data_min,data_max):
        Num_dict[i] = 0
        Index_list.append(i)

    df = pd.DataFrame(columns = columns, 
                   index = Index_list)
    df = df.fillna(0) 
    return df , Num_dict
def AverageAllData(inputdir,outputdir,filenum):
    sel_len = []
    flag = 0
    total = 0
    for outeridx in range(filenum):
        try:
            tempSelected_data = pd.read_csv(inputdir+"/out"+str(outeridx)+".csv")
            sel_len.append(len(tempSelected_data))

        except:
            #print(inputdir+str(outeridx)+"/SelectSpeed_"+lane+".csv not exist")
            continue
    if not sel_len:
        return
    sel_max , sel_min = GetMinMax(sel_len)
    
    columns = tempSelected_data.columns.values.tolist()
    sel_total_df , sel_num_dict = MakePdAndDict(sel_max,sel_min,columns)
    
    for outeridx in range(filenum):
        try:
            tempSelected_data = pd.read_csv(inputdir+"/out"+str(outeridx)+".csv")
        except:
            continue
        if flag ==0:
            if len(tempSelected_data) >= sel_min:
                Selected_data = tempSelected_data[:sel_min]

            for i in range(sel_min,len(tempSelected_data) ):
                sel_total_df.loc[i] += tempSelected_data.iloc[i]
                sel_num_dict[i] += 1
            flag = 1
            total += 1
        else:
            if len(tempSelected_data) >= sel_min:
                Selected_data += tempSelected_data[:sel_min]

            for i in range(sel_min,len(tempSelected_data) ):
                sel_total_df.loc[i] += tempSelected_data.iloc[i]
                sel_num_dict[i] += 1
            total += 1

    #avginst_data /= filenum
    Selected_data /= total

    for i in range(sel_min,sel_max):
        sel_total_df.loc[i] /=sel_num_dict[i]
        Selected_data = Selected_data.append(sel_total_df.loc[i])

    Selected_data.to_csv(outputdir+"/entropyavg.csv", index=False)

if __name__ == "__main__":
    inputdir = "entropycsv_result"
    outputdir = "entropyavg_result"
    if not os.path.exists(outputdir):
        os.makedirs(outputdir)
        
    AverageAllData(inputdir,outputdir,30)
    
    
    
    