import numpy as np
import cv2
import os

DIM=(324, 244)
K = np.array([[55.33391825477185, 0.0, 157.59360464729062], [0.0, 54.84168328799328, 149.84307839820286], [0.0, 0.0, 1.0]])
D = np.array([[0.2630630021513974], [0.05010380465347993], [0.030779363074071555], [0.008559784664570531]])

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
cap.set(cv2.CAP_PROP_FRAME_WIDTH,324)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,243)
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
	    frame_crop = frame #[30:-1,:,:]
            gray = cv2.cvtColor(frame_crop, cv2.COLOR_BGR2GRAY)
            # write the flipped frame
            # out.write(frame)
            frame_undistorted = undistort(gray, 1.0)
            print(frame_undistorted.shape)
            cv2.imshow('frame',frame_undistorted)

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
