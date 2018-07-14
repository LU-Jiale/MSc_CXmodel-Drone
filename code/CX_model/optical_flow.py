import numpy as np
import cv2
from numpy import linalg as LA

# compute calibration map matrixes
DIM=(1296, 972)
FRAME_DIM = (216, 162)
#K=np.array([[1440.318444287085, 0.0, 676.9511026584912], 
#           [0.0, 1456.4727144606293, 540.711667283094], [0.0, 0.0, 1.0]])
#D=np.array([[-0.8909302058344544], [3.1817023042732813], 
#            [-12.598093051063067], [17.641313727690882]])
K=np.array([[649.7237194130138, 0.0, 570.0013929289199], 
           [0.0, 627.6183259974277, 532.3632845985546], [0.0, 0.0, 1.0]])
D=np.array([[-0.1428222048947462], [0.22455805794512237], 
           [-0.2695633377157125], [0.1381009248014135]])
scaled_K = K * FRAME_DIM[0] / DIM[0]  # The values of K is to scale with image dimension.
scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
# This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. 
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, FRAME_DIM, np.eye(3), balance=1.0)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, FRAME_DIM, cv2.CV_16SC2)


def undistort(img):
    ''' undistort and crop frames from the fisheye image 
    '''
    dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    # crop images 
    return undistorted_img[70:200,18:-1]

def get_filter(fh, fw):
    ''' Generate match filter for optical flow computation, one for left 45 degree 
        one for right 45 degree 
    '''
    # filter for speed retrieval
    vertical_views = (np.arange(fh, dtype=float)-fh/2)/fh*(90.0/180.0*np.pi)
    horizontal_views = (np.arange(fw, dtype=float)-fw/2)/fw*(80.0/180.0*np.pi)
    D = np.ones([fh,fw,3])*-1
    D[:,:,0] = np.tan(vertical_views).reshape(fh, 1)
    D[:,:,1] = np.tan(horizontal_views)
    sin_theta = LA.norm(D[:,:,0:2], axis = 2) + 0.0000001
    mag_temp = LA.norm(D, axis = 2) + 0.0000001
    D /= mag_temp.reshape(fh,fw,1)
    a_l = a = np.array([-1/np.sqrt(2), 1/np.sqrt(2), 0])
    a_r = a = np.array([-1/np.sqrt(2), -1/np.sqrt(2), 0])
    left_filter = np.cross(np.cross(D,a_l),D)[:,:,0:2] #/ sin_theta.reshape(fh,fw,1)
    right_filter = np.cross(np.cross(D,a_r),D)[:,:,0:2] #/ sin_theta.reshape(fh,fw,1)
    return left_filter, right_filter

def get_speed(flow, left_filter, right_filter, elapsed_time):
    ''' calculate speeds from optical flow using match filters
    '''    
    mag = LA.norm(flow/left_filter, axis=2)
    mag[mag < 2.0] = 0  # filter out those noisy flow
    mag[mag > 0.0] = 1.0
    count = np.sum(mag)
    #print count
    weight = mag/(elapsed_time*count*100+1)
    weight = weight.reshape(weight.shape[0], weight.shape[1], 1)  # reshape for broadcasting
    sl = np.sum(flow * left_filter * weight)
    sr = np.sum(flow * right_filter * weight)
    return sl, sr

