import numpy as np
import sys, os, time
import logging, datetime
import cv2
import cx_rate, cx_basic
import central_complex
import dronekit
from graphics import draw_flow, frame_preprocess
from optical_flow import undistort, get_filter, get_speed
from central_complex import update_cells

# initialize logger
fname = str(datetime.datetime.now()).replace(':', '-') + '.log'
logging.basicConfig(filename=fname,level=logging.DEBUG)

# initialize CX model
cx = cx_rate.CXRate(noise = 0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)

# initialize camera and optical flow
frame_num = 0 
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,324)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,243)
cap.set(cv2.CAP_PROP_FPS, 30)
ret, frame1 = cap.read()
temp = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
prvs = undistort(temp, 1.0)
prvs = frame_preprocess(prvs, scale=1.0, crop_size = [0.0, 0.0])
(fh, fw) = prvs.shape
print("Frame size: {0}*{1}".format(fw,fh))
left_filter, right_filter = get_filter(fh, fw)

# connect to PX4
try:
    drone = dronekit.connect('/dev/ttyAMA0', baud = 57600, heartbeat_timeout=15)
except dronekit.APIException:
    logging.critical('Timeout! Fail to connect PX4')
    raise Exception('Timeout! Fail to connct PX4')
except:    
    logging.critical('Some other error!')
    raise Exception('Fail to connct PX4')


start_time = time.time()
for i in range(1500):    
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
    

    # updare cx_neurons
    velocity = np.array([sl, sr])
    tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=0, velocity=velocity, tb1=tb1, memory=memory, cx=cx)
    
    # logging
    logging.info('sl:{} sr:{} heading:{}, velocity:{}'.format(sl,sr,drone.heading,drone.velocity))

    prvs = next
    start_time = time.time()
angle, distance = cx.decode_cpu4(cpu4)
print((angle/np.pi) * 180, distance)
logging.info('Angle:{} Distance:{}'.format((angle/np.pi) * 180, distance))
drone.close()
cap.release()
cv2.destroyAllWindows()





