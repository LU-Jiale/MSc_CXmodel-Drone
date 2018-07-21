import os
import numpy as np
import time
import central_complex
import cx_rate
import cx_basic

def update_cells(heading, velocity, tb1, memory, cx, filtered_steps=0.0):
    """Generate activity for all cells, based on previous activity and current
    motion."""
    # Compass
    tl2 = cx.tl2_output(heading)
    cl1 = cx.cl1_output(tl2)
    tb1 = cx.tb1_output(cl1, tb1)

    # Speed
    flow = cx.get_flow(heading, velocity, filtered_steps)
    tn1 = cx.tn1_output(velocity)
    tn2 = cx.tn2_output(velocity)

    # Update memory for distance just travelled
    memory = cx.cpu4_update(memory, tb1, tn1, tn2)
    cpu4 = cx.cpu4_output(memory)

    # Steer based on memory and direction
    cpu1 = cx.cpu1_output(tb1, cpu4)
    motor = cx.motor_output(cpu1)
    return tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor

error_log_path = 'log/2018-07-19_07-47-37.log'
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

# initialize CX model
#cx = cx_basic.CXBasic()
cx = cx_rate.CXRate(noise=0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)

T = 200
velocity = np.ones([T,2], dtype=float)
headings = np.zeros(T, dtype=float)
headings[:] = -np.pi/2

for t in range(T): #len(speed_left)):
        speed = np.array([speed_left_real[t], speed_right_real[t]])
        
        tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=headings[t], velocity=velocity[t], tb1=tb1, memory=memory, cx=cx)

        angle, distance = cx.decode_cpu4(cpu4)
        angle_degree = angle/np.pi * 180
        print "Angle:%.2f  Distance:%.2f Motor:%.2f" % (angle_degree, distance, motor)
        #time.sleep(0.1)
