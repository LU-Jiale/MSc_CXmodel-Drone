import cv2
import sys
import numpy as np

#DIM=(648, 486)
#K=np.array([[246.35274583696963, -4.826428559211498, 323.93183074769644], [0.0, 243.4716435505204, 243.24494508428336], [0.0, 0.0, 1.0]])
#D=np.array([[-0.08634557881311974], [0.09465735860807668], [0.05719051783268446], [-0.005709516540189177]])
#K=np.array([[206.26480624709637, 0.0, 323.5], [0.0, 206.26480624709637, 242.5], [0.0, 0.0, 1.0]])
#K = np.array([[258.83903327097954, 0.0, 366.9420589448335], [0.0, 258.2501381667232, 266.2952450984071], [0.0, 0.0, 1.0]])
#D = np.array([[0.1515852898993269], [-0.4089304163756245], [0.41430881723992785], [-0.13253237501826373]])
DIM=(324, 244)
#K = np.array([[55.33391825477185, 0.0, 157.59360464729062], [0.0, 54.84168328799328, 149.84307839820286], [0.0, 0.0, 1.0]])
#D = np.array([[0.2630630021513974], [0.05010380465347993], [0.030779363074071555], [0.008559784664570531]])
K = np.array([[180.89915252967216, 0.0, 150.73638502215627], [0.0, 181.27765039282147, 165.99895975342446], [0.0, 0.0, 1.0]])
D = np.array([[0.5327427168832571], [-1.2346384996613664], [1.6864346258077718], [-1.0999691170149224]])


def undistort(img_path, balance=0.0, dim2=None, dim3=None):

    img = cv2.imread(img_path)
    print(img.shape)
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

    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    undistort(sys.argv[1], float(sys.argv[2]))
