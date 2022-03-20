#COST2與COST1的不同
# 1、 COST1只用當前當下速度  COST2是可以用LSTM，再把瞬時速度轉為每一輛車的平均速度
# 2、 COST1沒有DB  COST2有DB
# 3、 COST1沒有用LSTM COST2用到LSTM（並且是預測的是error）


# Attention:必須把Cost1 run完畢（睦昆建議可以先跑10次），才能run2.
# COST2run完，會產生檔案，然後用新檔案再run一次COst2
import datetime
import os
from pathlib import Path
import traceback
import numpy as np

def test():
    try:
        test = np.infty
        return int(test / 300)

    except Exception as e:
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    print(test())
    print("test")






