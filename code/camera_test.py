import sys
import numpy as np
import cv2
import os
from CX_model.optical_flow import Optical_flow

fw = 1296/2
fh = 972/2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,fw)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,fh)
cap.set(cv2.CAP_PROP_FPS, 30)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Frame size: {}*{}".format(fw, fh))

optflow = Optical_flow();

picture_num = 10
column_num = 0
if(cap.isOpened()):
    while(1):
        ret, frame = cap.read()
        if ret==True:
            frame = cv2.flip(frame,0)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # undistorte image
            gray = optflow.undistort(gray)
#            gray = gray[216:315,235:435]
            # draw lines on image for calibration angles
            fh,fw = gray.shape
            gray = cv2.line(gray,(1, column_num),(fw, column_num),(255,255,0),1)
            gray = cv2.line(gray,(1, int(fh/2)),(fw, int(fh/2)),(255,255,0),1)

            gray = cv2.line(gray,(column_num, 1),(column_num, fh),(255,255,0),1)
            gray = cv2.line(gray,(int(fw/2), 1),(int(fw/2), fh),(255,255,0),1)

            cv2.imshow('frame', cv2.resize(gray, (0, 0), fx=1, fy=1))
            ch = 0xFF & cv2.waitKey(1)
            if ch == ord('q'):
                break
            elif ch == ord('s'):
                picture_name = "image" + str(picture_num) + '.jpg'
                cv2.imwrite(picture_name,gray)
                picture_num = picture_num + 1
                print('Save frame: {}'.format(picture_name))
            elif ch == ord('='):
                column_num = (column_num + 1) % fw
                print('Now the column is: {}'.format(column_num))
            elif ch == ord('-'):
                column_num = (column_num - 1) % fw
                print('Now the column is: {}'.format(column_num))
        else:
            break
else:
    print('No camera!')
# Release everything if job is finished
cap.release()
cv2.destroyAllWindows()
