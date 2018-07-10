import sys
import numpy as np
import cv2
import os

DIM=(1296, 972)
#K=np.array([[732.3522413170597, 0.0, 665.8885698012476], [0.0, 729.0931362872735, 525.4529490954391], [0.0, 0.0, 1.0]])
#D=np.array([[-0.09586431729880242], [0.033758571018492424], [-0.16763638449016888], [0.11831395460567679]])
#K=np.array([[660.6098041503778, 0.0, 665.6546670454397], [0.0, 660.3604369721945, 511.57917049614656], [0.0, 0.0, 1.0]])
#D=np.array([[-0.08973017001349061], [0.014517673009189712], [-0.02365561646089509], [0.015926363249533296]])
K=np.array([[1440.318444287085, 0.0, 676.9511026584912], [0.0, 1456.4727144606293, 540.711667283094], [0.0, 0.0, 1.0]])
D=np.array([[-0.8909302058344544], [3.1817023042732813], [-12.598093051063067], [17.641313727690882]])

fw = 1296/4
fh = 972/4


def undistort(img, balance=1.0, dim2=None, dim3=None):

    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort

    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"

    if not dim2:
        dim2 = dim1

    if not dim3:
        dim3 = dim1

    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0

    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,fw)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,fh)
cap.set(cv2.CAP_PROP_FPS, 30)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Frame size: {}*{}".format(fw, fh))

picture_num = 10
column_num = 0
if(cap.isOpened()):
    while(1):
        ret, frame = cap.read()
        if ret==True:
            frame = cv2.flip(frame,0)
#            frame = cv2.flip(frame,1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # write the flipped frame
            #out.write(frame)
            # undistorte image
            gray = undistort(gray, 1.0)
#            gray = gray[50:-50,10:-10]

            # draw lines on image for calibration angles
            gray = cv2.line(gray,(column_num, 1),(column_num, fh),(255,255,0),1)
            gray = cv2.line(gray,(int(fw/2), 1),(int(fw/2), fh),(255,255,0),1)

            cv2.imshow('frame', cv2.resize(gray, (0, 0), fx=2, fy=2))
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
