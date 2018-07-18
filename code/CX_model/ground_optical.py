import numpy as np
import cv2
import cx_rate, cx_basic
import central_complex
import sys, os, time
import dronekit
import matplotlib.pyplot as plt
from graphics import draw_flow, frame_preprocess
from optical_flow import undistort, get_filter, get_speed
from central_complex import update_cells

# initialize CX model
cx = cx_rate.CXRate(noise = 0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)

# initialize camera
frame_num = 0 
cap = cv2.VideoCapture(sys.argv[1])

for i in range(250):              
    ret, frame1 = cap.read()      # Skip frames
    frame_num += 1
temp = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
prvs = undistort(temp)
prvs = frame_preprocess(prvs, scale=1.0, crop_size = [0.0, 0.0])
(fh, fw) = prvs.shape
print("Frame size: {0}*{1}".format(fw,fh))

# visulize computed speed 
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, sharey=True)
ax1.set(title='speed', ylabel='left')
ax2.set(xlabel='time (s)', ylabel='right')
x_axis = np.linspace(0, 100, num=100, endpoint=False)
speed_left = np.zeros(100)
speed_right = np.zeros(100)
plt.show()

# load log file
error_log_path = 'log/2018-07-18_14-02-55.679331.log'
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

distance_list = []
time_list = []
for i in range(len(navigation_info)):
    distance = model_info[i].split('Distance:')[-1].split(' ')[0]
    distance_list.append(float(distance))

    elapsed_time = model_info[i].split('elapsed_time:')[-1]
    time_list.append(float(elapsed_time))

left_filter, right_filter = get_filter(fh, fw)
while(1):    
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    frame_num += 1    
    temp = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    next = undistort(temp)
    next = frame_preprocess(next, scale=1.0, crop_size = [0.0, 0.0])
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.1, 0)
    
    # speed
    sl, sr = get_speed(flow, left_filter, right_filter, time_list[frame_num])

    # visulize computed speed 
    speed_left = np.roll(speed_left, -1)
    speed_left[-1] = (sl) # + np.sum(speed_left[1:3]))/4
    speed_right = np.roll(speed_right, -1)
    speed_right[-1] = (sr) # + np.sum(speed_right[1:3]))/4
    ax1.clear()
    ax2.clear()
    ax1.plot(x_axis, speed_left, 'r-')
    ax1.plot(x_axis, speed_right, 'b-')
    plt.draw()

    # updare cx_neurons
    velocity = np.array([sl, sr])
    tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=0, velocity=velocity, tb1=tb1, memory=memory, cx=cx)
    angle, distance = cx.decode_cpu4(cpu4)
    #print((angle/np.pi) * 180, distance)

    # show frames
    cv2.imshow('vedio', cv2.resize(draw_flow(next, flow), (0,0), fx=1.0, fy=1.0))
    #print('Frame number: ', frame_num)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break
    prvs = next
    start_time = time.time()
cap.release()
cv2.destroyAllWindows()





