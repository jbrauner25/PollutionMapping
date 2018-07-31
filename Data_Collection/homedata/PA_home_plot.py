import datetime
import scipy.io
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

data = scipy.io.loadmat('home_data3.mat').get('data')

pol = [(float(d[0].strip())+float(d[1].strip()))/2 for d in data]

interp = []
i = 0
while i < len(pol):
    if pol[i] != 0:
        interp.append(pol[i])
        i = i + 1
    else:
        point = 0
        for j in range(i, len(pol)):
            if pol[j] != 0:
                point = j + 1
                break
        #print(pol[i-1], pol[i], pol[point-1], pol[point])
        holeSize = point - i + 1
        for k in range(i, point):
            pol_est = pol[i-1] + (k-i+1)*(pol[point]-pol[i-1])/holeSize
            interp.append(pol_est)
        i = point

for p in pol:
    print(p)

#for p in interp:
#    print(p)

plt.plot(interp)
plt.show()


# pol = [d[0] for d in data]
# time = [data[d][1][:13] for d in range(len(data))]
# x_axis = range(len(pol))
# #time = [datetime.datetime.strptime(d, '%Y/%m/%d/%H/%M/%S').date() for d in time]
#
# #fig, ax = plt.subplots(1)
# #fig.autofmt_xdate()
# #plt.plot(time, pol)
# #xfmt = mdates.DateFormatter('%Y/%m/%d/%H/%M/%S')
# #ax.xaxis.set_major_formatter(xfmt)
#
#
# #plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%d %H:%M:%S'))
# #plt.gca().xaxis.set_major_locator(mdates.DayLocator())
# #plt.plot(time, pol)
# #plt.gcf().autofmt_xdate()
# #plt.show()
#
# plt.plot(pol)
# #plt.xticks(x_axis, time, rotation='vertical')
# #plt.margins(0.2)
# #plt.subplots_adjust(left=0.3, bottom=0.3)
# #plt.xlabel('Date and Time')
# #plt.ylabel('PM2.5 Particulate Mass (ug/m3)')
# plt.show()
