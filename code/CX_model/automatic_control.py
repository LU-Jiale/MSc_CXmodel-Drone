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
parser.add_argument('--recordto', default = 'no', 
                    help='Video name for recording, set to \'no\' to disable(default: no)')

args = parser.parse_args()
RECORDING = args.recordto

# initialize logger
fname = 'log/' + str(datetime.datetime.now()).replace(':', '-') + '.log'
logging.basicConfig(filename=fname,level=logging.DEBUG)

# initialize CX model
cx = cx_rate.CXRate(noise = 0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)

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
if RECORDING != 'no':
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(sys.argv[1],fourcc, 20.0, (fw,fh))
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
state = arm(drone)
logging.info(state)

start_time = time.time()
for i in range(100):
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    frame_num += 1
    frame_gray = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    next = undistort(frame_gray)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.1, 0)
    # speed
    elapsed_time = time.time() - start_time
    sl, sr = get_speed(flow, left_filter, right_filter, elapsed_time)

    # updare cx_neurons
    velocity = np.array([sl, sr])
    tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=drone.heading/180*np.pi, velocity=velocity, tb1=tb1, memory=memory, cx=cx)

    # write the frame
    if RECORDING != 'no':
        out.write(frame2)
    # logging
    logging.info('sl:{} sr:{} heading:{}, velocity:{}'.format(sl,sr,drone.heading,drone.velocity))

    prvs = next
    start_time = time.time()
    print('Elapsed time:%.5f'%elapsed_time)

angle, distance = cx.decode_cpu4(cpu4)
print((angle/np.pi) * 180, distance)
logging.info('Angle:{} Distance:{}'.format((angle/np.pi) * 180, distance))
drone.close()
if RECORDING != 'no':
    out.release()
cap.release()
cv2.destroyAllWindows()





