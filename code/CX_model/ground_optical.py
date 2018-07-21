import numpy as np
import cv2
import cx_rate, cx_basic
import central_complex
import sys, os, time
import dronekit
import PyQt5
import matplotlib
import matplotlib.pyplot as plt
from graphics import draw_flow, frame_preprocess
from optical_flow import Optical_flow
from central_complex import update_cells

# initialize CX model
cx = cx_rate.CXRate(noise = 0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)

# initialize camera
frame_num = 0 
cap = cv2.VideoCapture(sys.argv[1])
for i in range(10):              
    ret, frame1 = cap.read()      # Skip frames
    frame_num += 1
temp = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)

# intialise optical flow object
optflow = Optical_flow();
prvs = optflow.undistort(temp)
prvs = frame_preprocess(prvs, scale=1.0, crop_size = [0.0, 0.0])
(fh, fw) = prvs.shape
print("Frame size: {0}*{1}".format(fw,fh))

# log information
str = sys.argv[1]
error_log_path = 'log/'+str.split('.')[0].split('/')[1]+'.log'
with open(error_log_path) as f:
    data = f.read()
data = data.split('\n')
navigation_info = []
model_info = []
for i in range(len(data)/2):
    navigation_info.append(data[2*i])
    model_info.append(data[2*i+1])
time_list = []
for i in range(len(model_info)):
    elapsed_time = model_info[i].split('elapsed_time:')[-1]
    time_list.append(float(elapsed_time))
angle_list = np.zeros(len(model_info), dtype=float)
distance_list = np.zeros(len(model_info), dtype=float)

# visulize computed speed 
plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(3, sharey=False)
ax1.set(title='speed', ylabel='left')
ax2.set(ylabel='right')
ax3.set(xlabel='time (s)', ylabel='compare')
x_axis = np.linspace(0, len(time_list), num=len(time_list), endpoint=False)
speed_left = np.zeros(len(time_list))
speed_right = np.zeros(len(time_list))
plt.show()

left_filter, right_filter = optflow.get_filter(fh, fw)
for i in range(len(time_list)-frame_num-1):    
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    if not ret:
        break
    frame_num += 1    
    temp = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    next = optflow.undistort(temp)
    next = frame_preprocess(next, scale=1.0, crop_size = [0.0, 0.0])
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.1, 0)
    
    # speed
    sl, sr = optflow.get_speed(flow, left_filter, right_filter, time_list[frame_num])

    # visulize computed speed 
    speed_left = np.roll(speed_left, -1)
    speed_left[-1] = (sl) # + np.sum(speed_left[-3:-1]))/4
    speed_right = np.roll(speed_right, -1)
    speed_right[-1] = (sr) # + np.sum(speed_right[-3:-1]))/4
    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax1.plot(x_axis, speed_left, 'r-')
    ax2.plot(x_axis, speed_right, 'b-')
    ax3.plot(x_axis, speed_left, 'r-')
    ax3.plot(x_axis, speed_right, 'b-')
    plt.draw()

    # updare cx_neurons
    velocity = np.array([sl, sr])
    tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=100, velocity=velocity, tb1=tb1, memory=memory, cx=cx)
    angle, distance = cx.decode_cpu4(cpu4)
    angle_list[i] = angle/np.pi*180.0
    distance_list[i] = distance

    # show frames
    cv2.imshow('vedio', cv2.resize(draw_flow(next, flow), (0,0), fx=3.0, fy=3.0))
    #print('Frame number: ', frame_num)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break
    prvs = next
    start_time = time.time()

fig, (ax1, ax2) = plt.subplots(2, sharey=False)
ax1.plot(x_axis, angle_list, 'b-')
ax2.plot(x_axis, distance_list, 'b-')
plt.show()

cv2.waitKey()
cap.release()
cv2.destroyAllWindows()





