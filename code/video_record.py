import sys
import numpy as np
import cv2
import logging, datetime
import os, time
import dronekit

# initialize logger
fname = 'log/' + str(datetime.datetime.now()).replace(':', '-') + '.log'
logging.basicConfig(filename=fname,level=logging.DEBUG)\

# connect to PX4
try:
    drone = dronekit.connect('/dev/ttyAMA0', baud = 921600, heartbeat_timeout=15)
except dronekit.APIException:
    logging.critical('Timeout! Fail to connect PX4')
    raise Exception('Timeout! Fail to connct PX4')
except:
    logging.critical('Some other error!')
    raise Exception('Fail to connct PX4')

# initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 324)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,243)
cap.set(cv2.CAP_PROP_FPS, 30)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Frame size: {}*{}".format(fw, fh))
# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(sys.argv[1],fourcc, 20.0, (fw,fh))

start_time = time.time()
if(cap.isOpened()):
    while(1):
        ret, frame = cap.read()
        elapsed_time = time.time() - start_time
        start_time = time.time()
        if ret==True:
            frame = cv2.flip(frame,0)
            frame = cv2.flip(frame,1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # write the flipped frame
            out.write(frame)
            # logging
            logging.info('heading:{} velocity:{} position:{} elapsed_time:{}'
                         .format(drone.heading,drone.velocity,
                         drone.location.global_relative_frame, elapsed_time))
        else:
            break
else:
    print('No camera!')
# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
