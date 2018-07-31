import csv
import matplotlib.pyplot as plt
import datetime
import numpy as np

PA_filename = 'polTest_PA3.txt'
simple_filename = 'polTest_simp3.txt'
start_time = datetime.datetime.strptime('15:58:01', '%H:%M:%S')

with open(PA_filename) as f:
    reader = csv.reader(f, delimiter=",")
    full_data = list(reader)
    PA_data = []
    for p in full_data:
        if len(p[0]) == 66:
            PA_data.append(p)

with open(simple_filename) as f:
    reader = csv.reader(f, delimiter=",")
    simple_data = list(reader)

total_data = []
for PA in range(len(PA_data)-2):
    if len(PA_data[PA]) > 13:
        time = PA_data[PA][-1][1:]
        if(datetime.datetime.strptime(time, '%H:%M:%S') > start_time):
            next_index = '-1'
            for ind in range(PA+1, len(PA_data)):
                if len(PA_data[ind]) > 13:
                    next_index = ind
                    break
            if next_index != -1:
                for simp in range(len(simple_data)):
                    if simple_data[simp][-1][1:] == time:
                        simp_interval = []
                        for i in range(simp, len(simple_data)):
                            if datetime.datetime.strptime(simple_data[i][-1][1:], '%H:%M:%S') == datetime.datetime.strptime(PA_data[next_index][-1][1:], '%H:%M:%S'):
                                print(len(simp_interval))
                                total_data.append([np.mean(simp_interval), float(PA_data[PA][13][2:-1]), float(PA_data[PA][25][2:-1]), time])
                                break
                            else:
                                simp_interval.append(float(simple_data[i][0]))
                        break
            else:
                for simp in range(len(simple_data)):
                    if simple_data[simp][-1][1:] == time:
                        simp_interval = []
                        for i in range(simp, len(simple_data)):
                            simp_interval.append(float(simple_data[i][0]))
                        total_data.append([np.mean(simp_interval), float(PA_data[PA][13][2:-1]), float(PA_data[PA][25][2:-1]), time])
                        break


simps = [a[0] for a in total_data][:-1]
PAs = [(a[1]+a[2])/2 for a in total_data][1:]
diff = [simps[j]-PAs[j] for j in range(len(simps))]
print(np.mean(diff))

A, = plt.plot(simps, label='Simple Sensor')
B, = plt.plot(PAs, label='PurpleAir')
C, = plt.plot(diff, label='Difference')
plt.legend(handles=[A, B, C])
plt.xlabel('Time (80s interval)')
plt.ylabel('PM2.5 (ug/m3)')
plt.show()
