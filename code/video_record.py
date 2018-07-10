import sys
import numpy as np
import cv2
import os

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

if(cap.isOpened()):
    while(1):
        ret, frame = cap.read()
        if ret==True:
            frame = cv2.flip(frame,0)
            frame = cv2.flip(frame,1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # write the flipped frame
            out.write(frame)
        else:
            break
else:
    print('No camera!')
# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
