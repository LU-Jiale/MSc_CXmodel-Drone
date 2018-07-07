import numpy as np
import cv2
import os

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,648)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,486)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Frame size: {}*{}".format(fw, fh))
# Define the codec and create VideoWriter object
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('sample4.avi',fourcc, 20.0, (fw,fh))
picture_num = 0
if(cap.isOpened()):
    while(1):
        ret, frame = cap.read()
        if ret==True:
            frame = cv2.flip(frame,0)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # write the flipped frame
            # out.write(frame)

            cv2.imshow('frame',gray)

            ch = 0xFF & cv2.waitKey(1)
            if ch == ord('q'):
                break
            elif ch == ord('s'):
                picture_name = "frame" + str(picture_num) + '.png'
                cv2.imwrite(picture_name,gray)
                picture_num = picture_num + 1
                print('Save frame: {}'.format(picture_name))
        else:
            break
else:
    print('No camera!')
# Release everything if job is finished
cap.release()
#out.release()
cv2.destroyAllWindows()
