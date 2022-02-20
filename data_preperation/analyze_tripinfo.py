# %% [code]
import xml.etree.cElementTree as ET
import numpy as np
import csv

# %% [code] {"scrolled":true}
total_mean_duration = []
total_mean_routeLength = []
numbers_of_reroutingcars = []
times = []
# 車子總數
totalcar = 22000
Dirname = "test"
for i in range(0, 15):
    # 檔案路徑自行設定即可
    tree = ET.ElementTree(file='trip_info/' + Dirname + '/tripinfo' + str(i + 1) + '.xml')
    tree.getroot()
    root = tree.getroot()

    duration = []
    routeLength = []
    norerouting = []
    rerouting = []
    reroutingtimes = 0

    nore_duration = []
    nore_routeLength = []

    re_duration = []
    re_routeLength = []
    for elem in tree.iter(tag='tripinfo'):
        duration.append(float(elem.attrib['duration']) + float(elem.attrib['departDelay']))

        if elem.attrib['rerouteNo'] != "0":
            reroutingtimes += int(elem.attrib['rerouteNo'])

    total_mean_duration.append(np.mean(duration))
    times.append(reroutingtimes / totalcar)

    print("mean duration：", np.mean(duration))
    print("mean reroutingtimes: ", reroutingtimes / totalcar)
    print("--------------------")

print("total mean duration：", np.mean(total_mean_duration))
print("mean numbers of re-routing times：", np.mean(times))
print("mean duration std: ", np.std(total_mean_duration) / np.sqrt(len(total_mean_duration)))
print("reroute times std: ", np.std(times) / np.sqrt(len(times)))
# print("duplicate reroute car: " ,np.mean(carreroutingtimes))
# print("total mean routeLength：",np.mean(total_mean_routeLength))

# %% [code]
