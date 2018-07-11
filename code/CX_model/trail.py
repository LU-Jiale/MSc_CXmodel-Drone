import numpy as np
import cv2
import central_complex
import cx_rate
import cx_basic
import time
import sys
import dronekit
import os
import camera_calibration

def update_cells(heading, velocity, tb1, memory, cx, filtered_steps=0.0):
    """Generate activity for all cells, based on previous activity and current
    motion."""
    # Compass
    tl2 = cx.tl2_output(heading)
    cl1 = cx.cl1_output(tl2)
    tb1 = cx.tb1_output(cl1, tb1)

    # Speed
    flow = cx.get_flow(heading, velocity, filtered_steps)
    tn1 = cx.tn1_output(flow)
    tn2 = cx.tn2_output(flow)

    # Update memory for distance just travelled
    memory = cx.cpu4_update(memory, tb1, tn1, tn2)
    cpu4 = cx.cpu4_output(memory)

    # Steer based on memory and direction
    cpu1 = cx.cpu1_output(tb1, cpu4)
    motor = cx.motor_output(cpu1)
    return tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor

# connect to PX4
drone_connected = False
try:
    drone_connected = True
    drone = dronekit.connect('/dev/ttyAMA0', baud = 57600, heartbeat_timeout=15)
# API Error
except dronekit.APIException:
    drone_connected = False
    print 'Timeout!'
# Other error
except:
    drone_connected = False
    print 'Some other error!'


# initialize CX model
cx = cx_rate.CXRate(noise = 0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)

# initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1296/4)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,972/4)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fw_quarter = int(fw/4)
print("Frame size: {0}*{1}".format(fw,fh))

# filter for speed retrieval
row = np.linspace(0, fw, num=fw, endpoint=False)
match_filter = np.sin((row/fw -0.5)*np.pi)

# initalize frames
ret = False
while(ret==False):
    ret, frame = cap.read()
frame = cv2.flip(frame,0)
frame = cv2.flip(frame,1)
temp = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
prvs = camera_calibration.undistort(temp, 1.0)

while(1):
    start_time = time.time()
    # Image processing, compute optical flow
    ret, frame = cap.read()
    frame = cv2.flip(frame,0)
    frame = cv2.flip(frame,1)
    temp = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    next = camera_calibration.undistort(temp, 1.0)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    hori_flow = flow[:,:,0]
    
    left_frame_shift = fw/4
    frame_left = np.roll(hori_flow, -left_frame_shift, axis=1)
    frame_left[:,fw-left_frame_shift:fw-1] = 0
    right_frame_shift = fw/4
    frame_right = np.roll(hori_flow, right_frame_shift, axis=1)
    frame_right[:,0:right_frame_shift-1] = 0
    elapsed_time = time.time() - start_time
    # left speed
    mag = np.abs(frame_left)
    mag[mag < 1.0] = 0
    mag[mag > 0.0] = 1.0
    count = np.sum(mag)
    weight = mag/(elapsed_time*(count + 10000))
    sl = np.sum(frame_left * (match_filter[5:-5])*weight)
    # right speed
    mag = np.abs(frame_right)
    mag[mag < 1.0] = 0
    mag[mag > 0.0] = 1.0
    count = np.sum(mag)
    weight = mag/(elapsed_time*(count + 10000))
    sr = np.sum(frame_right * (match_filter[5:-5])*weight)

    # update CX model
    print(sl,sr)
    if drone_connected:
        tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
                heading=vehicle.heading, velocity=np.array([sl, sr]), tb1=tb1, 
                memory=memory, cx=cx)
        print(motor)
    # show video
    #leftF = np.roll(next, -fw_quarter, axis=1)
    #leftFlow = np.roll(flow, -fw_quarter, axis=1)
    #cv2.imshow('vedio', cv2.resize(draw_flow(next, flow), (0,0), fx=2, fy=2))
    #if cv2.waitKey(5) & 0xFF == ord('q'):
    #    break

    prvs = next

cap.release()
cv2.destroyAllWindows()





