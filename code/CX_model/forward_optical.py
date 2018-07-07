import numpy as np
import cv2
import central_complex
import cx_rate
import cx_basic
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

DIM=(648, 486)
#K=np.array([[246.35274583696963, -4.826428559211498, 323.93183074769644], [0.0, 243.4716435505204, 243.24494508428336], [0.0, 0.0, 1.0]])
D=np.array([[-0.08634557881311974], [0.09465735860807668], [0.05719051783268446], [-0.005709516540189177]])


K=np.array([[206.26480624709637, 0.0, 323.5], [0.0, 206.26480624709637, 242.5], [0.0, 0.0, 1.0]])
#D = np.array([[ 0.00001], [ 0.00001], [ 0.00001], [ 0.00001]])


def undistort(img, balance=1.0):

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


def draw_flow(img, flow, step=10):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T *10
    fy[:] = 0
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    #print(flow.shape)
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def update_cells(heading, velocity, tb1, memory, cx, filtered_steps=0.0):
    """Generate activity for all cells, based on previous activity and current
    motion."""
    # Compass
    tl2 = cx.tl2_output(heading)
    cl1 = cx.cl1_output(tl2)
    tb1 = cx.tb1_output(cl1, tb1)

    # Speed
    flow = cx.get_flow(heading, velocity, filtered_steps)
    tn1 = cx.tn1_output(flow)
    tn2 = cx.tn2_output(flow)

    # Update memory for distance just travelled
    memory = cx.cpu4_update(memory, tb1, tn1, tn2)
    cpu4 = cx.cpu4_output(memory)

    # Steer based on memory and direction
    cpu1 = cx.cpu1_output(tb1, cpu4)
    motor = cx.motor_output(cpu1)
    return tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor
'''
# connect to PX4
try:
    drone = dronekit.connect('/dev/ttyAMA0', baud = 57600, heartbeat_timeout=15)
# API Error
except dronekit.APIException:
    print 'Timeout!'
# Other error
except:
    print 'Some other error!'
'''

# initialize CX model
cx = cx_rate.CXRate(noise = 0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)



# initialize camera
cap = cv2.VideoCapture('sample1.avi')
cap.set(cv2.CAP_PROP_FRAME_WIDTH,200)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,130)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fw_quarter = int(fw/4)
print("Frame size: {0}*{1}".format(fw,fh))

# filter for speed retrieval
row = np.linspace(0, fw, num=fw, endpoint=False)
match_filter = np.sin((row/fw -0.5)*np.pi)

# visulize computed speed 
plt.ion()

fig, (ax1, ax2) = plt.subplots(2, sharey=True)
ax1.set(title='speed', ylabel='left')
ax2.set(xlabel='time (s)', ylabel='right')
speed_left = np.zeros_like(row)
speed_right = np.zeros_like(row)
plt.show()

ret, frame1 = cap.read()
prv = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
prvs = undistort(prv)
hsv = np.zeros_like(prvs)
hsv[...,1] = 255


while(1):
    start_time = time.time()
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    temp = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    next = undistort(temp)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    hori_flow = flow[:,:,0]
    
    frame_left = np.roll(hori_flow, -fw_quarter, axis=1)
    frame_left[:,fw-fw_quarter:fw-1] = 0
    frame_right = np.roll(hori_flow, fw_quarter, axis=1)
    frame_right[:,0:fw_quarter-1] = 0
    #elapsed_time = time.time() - start_time
    mag = np.abs(hori_flow)
    mag[mag < 1] = 0
    weight = mag/(np.sum(mag)+1000)
    sl = np.sum(frame_left * match_filter*weight)
    sr = np.sum(frame_right * match_filter*weight)
    # visulize computed speed 
    speed_left = np.roll(speed_left, 1)
    speed_left[0] = (sl + np.sum(speed_left[1:3]))/4
    speed_right = np.roll(speed_right, 1)
    speed_right[0] = sr
    ax1.clear()
    ax2.clear()
    ax1.plot(row, speed_left, 'k-')
    ax2.plot(row, speed_right, 'k-')
    plt.draw()

    # show video
    #leftF = np.roll(next, -fw_quarter, axis=1)
    #leftFlow = np.roll(flow, -fw_quarter, axis=1)
    cv2.imshow('vedio', cv2.resize(draw_flow(next, flow), (0,0), fx=2, fy=2))
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

    print(np.sum(mag),np.average(mag))
    prvs = next
cap.release()
cv2.destroyAllWindows()





