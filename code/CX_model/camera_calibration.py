import numpy as np
import cv2

#DIM=(324, 244)
#K = np.array([[180.78947447124168, 0.0, 169.46499593846872], [0.0, 179.24415780126137, 131.1646493611888], [0.0, 0.0, 1.0]])
#D = np.array([[-0.31585338591457013], [0.7261127065557235], [-0.9929549962890939], [0.46877308948619656]])
#D = np.array([[0.2630630021513974], [0.05010380465347993], [0.030779363074071555], [0.008559784664570531]])
DIM=(1296, 972)
K=np.array([[1440.318444287085, 0.0, 676.9511026584912], [0.0, 1456.4727144606293, 540.711667283094], [0.0, 0.0, 1.0]])
D=np.array([[-0.8909302058344544], [3.1817023042732813], [-12.598093051063067], [17.641313727690882]])


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
    return undistorted_img[30:-30,5:-5]
