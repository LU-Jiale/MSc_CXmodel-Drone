import numpy as np
import os
import glob
import matplotlib.pyplot as plt

error_log_path = 'CX_model/log/2018-07-19_07-47-37.log'
with open(error_log_path) as f:
    data = f.read()
data = data.split('\n')
#del data[0]

navigation_info = []
model_info = []

for i in range(len(data)/2):
    navigation_info.append(data[2*i])
    model_info.append(data[2*i+1])
print navigation_info[0]
print model_info[0]

speed_left = []
speed_left_real = []
speed_right = []
speed_right_real = []

heading_list = []

velocity_list = []
alt_list = []
lat_list = []
lon_list = []
time_list = []
distance_list = []
distance_list2 = []
angle_list1 = []
angle_list2 = []
for i in range(len(navigation_info)):
    sl=navigation_info[i].split(' ')[0].split(':')[-1]
    sr=navigation_info[i].split(' ')[1].split(':')[-1]
    speed_left.append(float(sl))
    speed_right.append(float(sr))

    heading = float(navigation_info[i].split(' ')[2].split(':')[-1])
    heading_list.append(heading)

    velocity = navigation_info[i].split('[')[1].split(']')[0].split(', ')
    velocity = [float(j) for j in velocity]
    velocity_list.append((velocity))
    
    left_real = (velocity[0]*np.cos(heading/180.0*np.pi-np.pi/4) + 
                velocity[1]*np.cos(heading/180.0*np.pi-np.pi/4-np.pi/2))
    speed_left_real.append(left_real/10.0)
    right_real = (velocity[0]*np.cos(heading/180.0*np.pi+np.pi/4) + 
                velocity[1]*np.cos(heading/180.0*np.pi+np.pi/4-np.pi/2))
    speed_right_real.append(right_real/10.0)

    alt = navigation_info[i].split('alt=')[-1]
    alt_list.append(float(alt))
    lat = navigation_info[i].split('lat=')[-1].split(',')[0]
    lon = navigation_info[i].split('lon=')[-1].split(',')[0]
    lat_list.append(float(lat))
    lon_list.append(float(lon))

    elapsed_time = model_info[i].split('elapsed_time:')[-1]
    time_list.append(float(elapsed_time))
    distance = model_info[i].split('Distance_optical:')[1].split(' ')[0]
    distance_list.append(float(distance)/100)
    
    distance = model_info[i].split('Distance_optical:')[-1].split(' ')[0]
    distance_list2.append(float(distance)/100)

    angle = model_info[i].split('Angle_optical:')[1].split(' ')[0]
    angle_list1.append(float(angle))
    angle = model_info[i].split('Angle_optical:')[-1].split(' ')[0]
    angle_list2.append(float(angle)/np.pi*180)

print("Data number:", len(heading_list))



fig, (ax1, ax2, ax3) = plt.subplots(3, sharey=True)
ax1.set(title='speed', ylabel='left')
ax2.set(xlabel='', ylabel='right')
ax2.set(xlabel='time (s)', ylabel='Distance')
x_axis = np.linspace(0, len(navigation_info), num=len(navigation_info), endpoint=False)

ax1.plot(x_axis, speed_left, 'r-')
ax1.plot(x_axis, speed_left_real, 'b-')
ax2.plot(x_axis, speed_right, 'r-')
ax2.plot(x_axis, speed_right_real, 'b-')
ax3.plot(x_axis, distance_list, 'r-')
ax3.plot(x_axis, distance_list2, 'b-')

fig2, (ax4, ax5) = plt.subplots(2, sharey=True)
ax4.plot(x_axis, angle_list1, 'r-')
ax4.plot(x_axis, angle_list2, 'b-')
ax5.plot(x_axis, heading_list, 'r-')
plt.draw()
plt.show()
