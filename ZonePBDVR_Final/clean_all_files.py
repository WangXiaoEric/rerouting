import os  # os是用来切换路径和创建文件夹的。
from shutil import copy  # shutil 是用来复制黏贴文件的

import shutil


if __name__ == "__main__":
    delete_dir_list = []
    delete_dir_list.append("csv_result\out0")
    delete_dir_list.append("finalavg_result")
    delete_dir_list.append("finalZ_result")
    delete_dir_list.append("interavg_result\out0")
    delete_dir_list.append("interZ_result\out0")
    delete_dir_list.append("occtxt_result\out0")



    for dir_name in delete_dir_list:  # 遍历pathDir下的所有文件filename
        shutil.rmtree(dir_name)