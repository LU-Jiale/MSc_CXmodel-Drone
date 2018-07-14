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

for i in range(1):              
    ret, frame1 = cap.read()      # Skip frames
    frame_num += 1
temp = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
prvs = undistort(temp, 1.0)
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


left_filter, right_filter = get_filter(fh, fw)
start_time = time.time()
while(1):    
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    frame_num += 1    
    temp = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    next = undistort(temp, 1.0)
    next = frame_preprocess(next, scale=1.0, crop_size = [0.0, 0.0])
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.1, 0)
    
    # speed
    elapsed_time = time.time() - start_time
    sl, sr = get_speed(flow, left_filter, right_filter, elapsed_time)

    # visulize computed speed 
    speed_left = np.roll(speed_left, 1)
    speed_left[0] = (sl) # + np.sum(speed_left[1:3]))/4
    speed_right = np.roll(speed_right, 1)
    speed_right[0] = (sr) # + np.sum(speed_right[1:3]))/4
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





