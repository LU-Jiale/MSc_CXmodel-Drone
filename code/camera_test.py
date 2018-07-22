
import sys
import numpy as np
import cv2
import os
from CX_model.optical_flow import Optical_flow, FRAME_LARGE, FRAME_SMALL

fw = FRAME_LARGE[0]/2
fh = FRAME_LARGE[1]/2
scale = 324.0/fw

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,fw)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,fh)
cap.set(cv2.CAP_PROP_FPS, 30)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Frame size: {}*{}".format(fw, fh))

# compute calibration map matrixes
DIM=(1296, 972)
#K=np.array([[1440.318444287085, 0.0, 676.9511026584912], [0.0, 1456.4727144606293, 540.711667283094], [0.0, 0.0, 1.0]])
#D=np.array([[-0.8909302058344544], [3.1817023042732813], [-12.598093051063067], [17.641313727690882]])
K=np.array([[743.8040173335771, 0.0, 647.9940524434143], [0.0, 728.8909350192504, 485.7950206412609], [0.0, 0.0, 1.0]])
D=np.array([[-0.20926662485054526], [-0.04800755535197234], [0.26419146114701453], [-0.1540385750579161]])
frame_dim = (fw,fh)
scaled_K = K * fw / DIM[0]  # The values of K is to scale with image dimension.
scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
# This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. 
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, frame_dim, np.eye(3), balance=1.0)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, frame_dim, cv2.CV_16SC2)

def undistort(img):
    ''' undistort and crop frames from the fisheye image 
        for resolution [648, 486], ceter at [333,262], angle range [-65,65,-14:14]
        for resolution []
    '''
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], \
           "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    undistorted_img = cv2.remap(img, map1, map2, interpolation= \
                                cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    # crop images 
    fh_s = int(0.164*frame_dim[1])
    fh_e = int(0.758*frame_dim[1])
    fw_s = int(0.15*frame_dim[0])
    fw_e = int(0.85*frame_dim[0])
    return undistorted_img[fh_s:fh_e, fw_s:fw_e] 


picture_num = 100
column_num = 0
if(cap.isOpened()):
    while(1):
        ret, frame = cap.read()
        if ret==True:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # undistorte image
            gray = undistort(gray)

            # draw lines on image for calibration angles
            fh,fw = gray.shape

            gray = cv2.line(gray,(1, column_num),(fw, column_num),(255,255,0),1)
            gray = cv2.line(gray,(1, int(fh/2)),(fw, int(fh/2)),(255,255,0),1)
            gray = cv2.line(gray,(column_num, 1),(column_num, fh),(255,255,0),1)
            gray = cv2.line(gray,(int(fw/2), 1),(int(fw/2), fh),(255,255,0),1)

            cv2.imshow('frame', cv2.resize(gray, (0, 0), fx=scale, fy=scale))
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
