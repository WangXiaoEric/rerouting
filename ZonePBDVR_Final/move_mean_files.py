import os  # os是用来切换路径和创建文件夹的。
from shutil import copy  # shutil 是用来复制黏贴文件的
import shutil


if __name__ == "__main__":
    file_path = "interZ_result\out0"  # 想拆分的文件夹所在路径,也就是一大堆文件所在的路径
    save_dir = "data\meanZ"  # save_dir 是想把复制出来的文件存放在的路径

    shutil.rmtree(save_dir)

    # 获取 file_path 下的文件和文件夹列表
    # 因为 file_path 里面没有文件夹，所以不处理有文件夹的情况
    pathDir = os.listdir(file_path)  # os.listdir(file_path) 是获取指定路径下包含的文件或文件夹列表
    for filename in pathDir:  # 遍历pathDir下的所有文件filename
        print(filename)
        from_path = os.path.join(file_path, filename)  # 旧文件的绝对路径(包含文件的后缀名)
        to_path = save_dir  # 新文件的绝对路径

        if not os.path.isdir(to_path):  # 如果 to_path 目录不存在，则创建
            os.makedirs(to_path)
        copy(from_path, to_path)  # 完成复制黏贴