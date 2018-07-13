import numpy as np
import cv2

DIM=(1296, 972)
#K=np.array([[1440.318444287085, 0.0, 676.9511026584912], [0.0, 1456.4727144606293, 540.711667283094], [0.0, 0.0, 1.0]])
#D=np.array([[-0.8909302058344544], [3.1817023042732813], [-12.598093051063067], [17.641313727690882]])
K=np.array([[649.7237194130138, 0.0, 570.0013929289199], [0.0, 627.6183259974277, 532.3632845985546], [0.0, 0.0, 1.0]])
D=np.array([[-0.1428222048947462], [0.22455805794512237], [-0.2695633377157125], [0.1381009248014135]])

def undistort(img, balance=1.0, dim2=None, dim3=None):

    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort

    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"

    if not dim2:
        dim2 = dim1

    if not dim3:
        dim3 = dim1

    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0

    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. 
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # crop images 
    crop_size = int(dim1[0] / 10)
    return undistorted_img[70:200,18:-1]
