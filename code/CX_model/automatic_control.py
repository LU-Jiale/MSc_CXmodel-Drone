

import numpy as np
import sys, os, time
import logging, datetime
import argparse
import cv2
import cx_rate, cx_basic
import central_complex
import dronekit
from graphics import draw_flow, frame_preprocess
from optical_flow import undistort, get_filter, get_speed, FRAME_DIM
from central_complex import update_cells
from drone_basic import arm, arm_and_takeoff

# command line arguments halder
parser = argparse.ArgumentParser(description='CX model navigation.')
parser.add_argument('--recording', default = 'no', 
                    help='Recoding option, true or false(default: false)')

args = parser.parse_args()
RECORDING = args.recording

# initialize logger
time_string = str(datetime.datetime.now()).replace(':', '-').replace(' ', '_').split('.')[0]
fname = 'log/' + time_string + '.log'
logging.basicConfig(filename=fname,level=logging.DEBUG)

# initialize CX models
cx_optical = cx_rate.CXRate(noise = 0)
tb1_optical = np.zeros(central_complex.N_TB1)
memory_optical = 0.5 * np.ones(central_complex.N_CPU4)

cx_gps = cx_rate.CXRate(noise = 0)
tb1_gps = np.zeros(central_complex.N_TB1)
memory_gps = 0.5 * np.ones(central_complex.N_CPU4)
cpu4_gps = np.zeros(16)

# initialize camera and optical flow
frame_num = 0
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_DIM[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_DIM[1])
cap.set(cv2.CAP_PROP_FPS, 30)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Frame size: {}*{}".format(fw, fh))
# Define the codec and create VideoWriter object
if RECORDING == 'true':
    fname = 'video/' + time_string + '.avi'
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(fname,fourcc, 20.0, (fw,fh))
if not cap.isOpened():
    logging.info('Camera not connected!')
    raise Exception('Camera not connected!')

# initialize optical flow
ret, frame1 = cap.read()
temp = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
prvs = undistort(temp)
(fh, fw) = prvs.shape
print("Frame size: {0}*{1}".format(fw,fh))
left_filter, right_filter = get_filter(fh, fw)

# connect to PX4 and arm
try:
    drone = dronekit.connect('/dev/ttyAMA0', baud = 921600, heartbeat_timeout=15)
except dronekit.APIException:
    logging.critical('Timeout! Fail to connect PX4')
    raise Exception('Timeout! Fail to connct PX4')
except:
    logging.critical('Some other error!')
    raise Exception('Fail to connct PX4')
#state = arm(drone)
#logging.info(state)

#while drone.mode.name != "MISSION":
#    print "Waiting for the mission mode."
#    time.sleep(2)

start_time = time.time()
print "Start to update CX model, switch mode to end"
#while drone.mode.name == "MISSION":
while True:
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    frame_num += 1
    frame_gray = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    next = undistort(frame_gray)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.1, 0)
    # speed
    elapsed_time = time.time() - start_time
    start_time = time.time()
    sl, sr = get_speed(flow, left_filter, right_filter, elapsed_time)

    # update CX neurons
    drone_heading = drone.heading/180*np.pi
    velocity = np.array([sl, sr])
    __, __, tb1_optical, __, __, memory_optical, cpu4_optical, __, motor_optical = \
            update_cells(heading=drone_heading, velocity=velocity, tb1=tb1_optical, \
                         memory=memory_optical, cx=cx_optical)

    velocity = drone.velocity
    if velocity[0]:
        left_real = (velocity[0]*np.cos(drone_heading-np.pi/4) + \
                     velocity[1]*np.cos(drone_heading-np.pi/4-np.pi/2))
        right_real = (velocity[0]*np.cos(drone_heading+np.pi/4) + \
                      velocity[1]*np.cos(drone_heading+np.pi/4-np.pi/2))
        velocity = np.array([left_real, right_real])
        __, __, tb1_gps, __, __, memory_gps, cpu4_gps, __, motor_gps = \
                update_cells(heading=drone_heading, velocity=velocity, \
                             tb1=tb1_gps, memory=memory_gps, cx=cx_gps)

    # write the frame for later recheck
    if RECORDING == 'true':
        out.write(frame2)
    # logging
    logging.info('sl:{} sr:{} heading:{} velocity:{} position:{}'.format(
                sl,sr,drone.heading,drone.velocity, drone.location.global_relative_frame))
    angle_optical, distance_optical = cx_optical.decode_cpu4(cpu4_optical)
    angle_gps, distance_gps = cx_gps.decode_cpu4(cpu4_gps)
    logging.info('Angle_optical:{} Distance_optical:{} Angle_gps:{} Distance_gps:{} \
                 elapsed_time:{}'.format((angle_optical/np.pi)*180, distance_optical, \
                 (angle_gps/np.pi)*180, distance_gps, elapsed_time))

    prvs = next
#    print('Elapsed time:%.5f'%elapsed_time)

print "Mission ended or stoppped. The final results of CX model based on optcial flow is:"
print((angle_optical/np.pi) * 180, distance_optical)
drone.close()
if RECORDING == 'true':
    out.release()
cap.release()
cv2.destroyAllWindows()
