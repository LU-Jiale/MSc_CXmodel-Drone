import numpy as np
import cv2
from numpy import linalg as LA

# compute calibration map matrixes
DIM=(1296, 972)
FRAME_LARGE= (648, 486)
K=np.array([[1440.318444287085, 0.0, 676.9511026584912], 
           [0.0, 1456.4727144606293, 540.711667283094], [0.0, 0.0, 1.0]])
D=np.array([[-0.8909302058344544], [3.1817023042732813], 
            [-12.598093051063067], [17.641313727690882]])

scaled_K = K * FRAME_LARGE[0] / DIM[0]  # The values of K is to scale with image dimension.
scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
# This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. 
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, FRAME_LARGE, np.eye(3), balance=1.0)
_map1, _map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, FRAME_LARGE, cv2.CV_16SC2)

FRAME_SMALL = (216, 162)
K=np.array([[649.7237194130138, 0.0, 570.0013929289199], 
           [0.0, 627.6183259974277, 532.3632845985546], [0.0, 0.0, 1.0]])
D=np.array([[-0.1428222048947462], [0.22455805794512237], 
           [-0.2695633377157125], [0.1381009248014135]])
scaled_K = K * FRAME_SMALL[0] / DIM[0]  # The values of K is to scale with image dimension.
scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
# This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. 
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, FRAME_SMALL, np.eye(3), balance=1.0)
_map3, _map4 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, FRAME_SMALL, cv2.CV_16SC2)


class Optical_flow():

    def __init__(self):
        self.speed_left_buffer = np.array([0, 0, 0, 0], dtype=float)
        self.speed_right_buffer = np.array([0, 0, 0, 0], dtype=float)
        self.map1 = _map1
        self.map2 = _map2
        self.map3 = _map3
        self.map4 = _map4
        self.accmax = 0.2

    def undistort1(self, img):
        ''' undistort and crop frames from the fisheye image 
            for resolution [648, 486], ceter at [333,262], angle range [-25,25,-14:14]
            for resolution []
        '''
        dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
        assert dim1[0]/dim1[1] == DIM[0]/DIM[1], \
               "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        undistorted_img = cv2.remap(img, self.map1, self.map2, interpolation= \
                                    cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # crop images 
        return undistorted_img [212:312, 233:433] 


    def undistort2(self, img):
        ''' undistort and crop frames from the fisheye image 
            for resolution [648, 486], ceter at [333,262], angle range [-25,25,-14:14]
        '''
        dim1 = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
        assert dim1[0]/dim1[1] == DIM[0]/DIM[1], \
               "Image to undistort needs to have same aspect ratio as the ones used in calibration"
        undistorted_img = cv2.remap(img, self.map3, self.map4, interpolation= \
                                    cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # crop images 
        return undistorted_img #[21:118, 18:-1]


    def get_filter(self, fh, fw):
        ''' Generate match filter for optical flow computation, one for left 45 degree 
            one for right 45 degree 
        '''
        # filter for speed retrieval
        vertical_views = (np.arange(fh, dtype=float)-fh/2)/fh*(90.0/180.0*np.pi)
        horizontal_views = (np.arange(fw, dtype=float)-fw/2)/fw*(160.0/180.0*np.pi)
        D = np.ones([fh,fw,3])*-1
        D[:,:,0] = np.tan(vertical_views).reshape(fh, 1)
        D[:,:,1] = np.tan(horizontal_views)
        sin_theta = LA.norm(D[:,:,0:2], axis = 2) + 0.0000001
        mag_temp = LA.norm(D, axis = 2) + 0.0000001
        D /= mag_temp.reshape(fh,fw,1)
        a_l = np.array([1/np.sqrt(2), -1/np.sqrt(2), 0])
        a_r = np.array([1/np.sqrt(2), 1/np.sqrt(2), 0])
        left_filter = np.cross(np.cross(D,a_l),D)[:,:,0:2] / sin_theta.reshape(fh,fw,1)
        right_filter = np.cross(np.cross(D,a_r),D)[:,:,0:2] / sin_theta.reshape(fh,fw,1)
        return left_filter, right_filter


    def cart2pol(self, x, y):
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        return(rho, phi)


    def pol2cart(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)


    def get_speed(self, flow, left_filter, right_filter, elapsed_time):
        ''' calculate speeds from optical flow using match filters
        '''    
        mag = LA.norm(flow/left_filter, axis=2)
    
        mag[mag < 0.5] = 0  # filter out those noisy flow
        mag[mag > 0.0] = 1.0
        count = np.sum(mag)
    
        (rho, phi) = self.cart2pol(flow[:,:,0], flow[:,:,1])
        mean = np.mean(phi)
        phi_diff = phi-mean
        phi_diff[np.abs(phi_diff)>np.pi/2] = np.pi/2
        #print count
        weight = mag/(elapsed_time*count*10+1) #* np.cos(phi_diff)
        weight = weight.reshape(weight.shape[0], weight.shape[1], 1)  # reshape for broadcasting
        self.speed_left_buffer = np.roll(self.speed_left_buffer, 1)
        self.speed_left_buffer[0] = np.sum(flow * left_filter * weight)
        self.speed_right_buffer = np.roll(self.speed_right_buffer, 1)
        self.speed_right_buffer[0] = np.sum(flow * right_filter * weight)
        sl = np.mean(self.speed_left_buffer)
        sl = np.max([np.min([self.speed_left_buffer[1]+self.accmax, sl]), self.speed_left_buffer[1]-self.accmax])
        sr = np.mean(self.speed_right_buffer)
        sr = np.max([np.min([self.speed_right_buffer[1]+self.accmax, sr]), self.speed_right_buffer[1]-self.accmax])
        self.speed_left_buffer[0] = sl
        self.speed_right_buffer[0] = sr
        return sl, sr
    
