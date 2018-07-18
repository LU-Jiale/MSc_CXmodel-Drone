import numpy as np
import os
import glob
import matplotlib.pyplot as plt

error_log_path = 'CX_model/log/2018-07-17_05-51-52.904135.log'
with open(error_log_path) as f:
    data = f.read()
data = data.split('\n')
del data[0]

navigation_info = []
model_info = []

for i in range(len(data)/2):
    navigation_info.append(data[2*i])
    model_info.append(data[2*i+1])
print navigation_info[0]
print model_info[0]

speed_left = []
speed_right = []
heading_list = []
for i in range(len(navigation_info)):
    sl=navigation_info[i].split(' ')[0].split(':')[-1]
    sr=navigation_info[i].split(' ')[1].split(':')[-1]
    speed_left.append(float(sl))
    speed_right.append(float(sr))
    heading = navigation_info[i].split(' ')[2].split(':')[-1]
    heading_list.append(int(heading))

fig, (ax1, ax2) = plt.subplots(2, sharey=True)
ax1.set(title='speed', ylabel='left')
ax2.set(xlabel='time (s)', ylabel='right')
x_axis = np.linspace(0, 100, num=len(navigation_info), endpoint=False)

ax1.plot(x_axis, speed_left, 'r-')
ax2.plot(x_axis, speed_right, 'b-')
plt.draw()
plt.show()
