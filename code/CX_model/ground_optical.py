import numpy as np
import cv2
import central_complex
import cx_rate
import cx_basic
import time
import sys
import dronekit
import os
import camera_calibration
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from numpy import linalg as LA

home = os.environ['HOME']
if home.split('/')[-1] == 'pi':
    show_frames = False
else:
    show_frames = True

def draw_flow(img, flow, step=10):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T *6
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (255, 255, 255))
    #print(flow.shape)
    #for (x1, y1), (x2, y2) in lines:
        #cv2.circle(vis, (x1, y1), 1, (255, 255, 255), -1)
    return vis

def frame_preprocess(img, scale=1.0, crop_size = [0.0, 0.2]):
    '''
    Scale and crop images, crop size is the propotion (fh_p, fw_p) of image
    that need to be cropped away (from the border).
    '''    
    img = cv2.resize(img, (0,0), fx=scale, fy=scale)
    dim = img.shape
    crop_width = int(dim[0]*crop_size[0])
    crop_height = int(dim[1]*crop_size[1])
    return img[crop_width:(dim[0]-crop_width), crop_height:(dim[1]-crop_height)]


def update_cells(heading, velocity, tb1, memory, cx, filtered_steps=0.0):
    """Generate activity for all cells, based on previous activity and current
    motion."""
    # Compass
    tl2 = cx.tl2_output(heading)
    cl1 = cx.cl1_output(tl2)
    tb1 = cx.tb1_output(cl1, tb1)

    # Speed
    tn1 = cx.tn1_output(velocity)
    tn2 = cx.tn2_output(velocity)

    # Update memory for distance just travelled
    memory = cx.cpu4_update(memory, tb1, tn1, tn2)
    cpu4 = cx.cpu4_output(memory)

    # Steer based on memory and direction
    cpu1 = cx.cpu1_output(tb1, cpu4)
    motor = cx.motor_output(cpu1)
    return tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor

# connect to PX4
try:
    drone = dronekit.connect('/dev/ttyAMA0', baud = 57600, heartbeat_timeout=15)
# API Error
except dronekit.APIException:
    print 'Timeout!'
# Other error
except:
    print 'Some other error!'


# initialize CX model
cx = cx_rate.CXRate(noise = 0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)



# initialize camera
frame_num = 0 
cap = cv2.VideoCapture(sys.argv[1])
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,200)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,130)
#fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


for i in range(1):              
    ret, frame1 = cap.read()      # Skip frames
    frame_num += 1
temp = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
prvs = camera_calibration.undistort(temp, 1.0)
prvs = frame_preprocess(prvs, scale=1.0, crop_size = [0.0, 0.0])
(fh, fw) = prvs.shape
print("Frame size: {0}*{1}".format(fw,fh))

# visulize computed speed 
plt.ion()
fig, (ax1, ax2) = plt.subplots(2, sharey=True)
ax1.set(title='speed', ylabel='left')
ax2.set(xlabel='time (s)', ylabel='right')
x_axis = np.linspace(0, 100, num=100, endpoint=False)
speed_left = np.zeros(100)
speed_right = np.zeros(100)
plt.show()

# filter for speed retrieval
vertical_views = (np.arange(fh, dtype=float)-fh/2)/fh*(100.0/180.0*np.pi)
horizontal_views = (np.arange(fw, dtype=float)-fw/2)/fw*(100.0/180.0*np.pi)

D = np.zeros([fh,fw,3])
for i in range(fh):
    for j in range(fw):
        D[i,j]=np.array([np.tan(vertical_views[i]), np.tan(horizontal_views[j]),-1])
        D[i,j] /= LA.norm(D[i,j])

a = np.array([0, 1, 0])
matched_filter = np.cross(np.cross(D,a),D)[:,:,0:2]

rho = LA.norm(matched_filter, axis=2)
phi = np.arctan2(matched_filter[:,:,0], matched_filter[:,:,1])
phi_l = phi + np.pi/4
phi_r = phi - np.pi/4
left_filter = np.zeros([fh,fw,2])
left_filter[:,:,0]=rho * np.cos(phi_l)
left_filter[:,:,1]=rho * np.sin(phi_l)
right_filter = np.zeros([fh,fw,2])
right_filter[:,:,0]=rho * np.cos(phi_r)
right_filter[:,:,1]=rho * np.sin(phi_r)

start_time = time.time()
while(1):    
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    frame_num += 1    
    temp = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    next = camera_calibration.undistort(temp, 1.0)
    next = frame_preprocess(next, scale=1.0, crop_size = [0.0, 0.0])
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.1, 0)
    
    # speed
    mag = LA.norm(flow/matched_filter, axis=2)
    mag[mag < 1.0] = 0
    mag[mag > 0.0] = 1.0
    count = np.sum(mag)
    elapsed_time = time.time() - start_time
    weight_e = mag/((0.034+elapsed_time)*count*100+1)
    weight = np.zeros_like(flow)
    weight[:,:,0] = weight_e
    weight[:,:,1] = weight_e
    sl = np.sum(flow * (left_filter)*weight)
    sr = np.sum(flow * (-right_filter)*weight)
    
    # visulize computed speed 
    speed_left = np.roll(speed_left, 1)
    speed_left[0] = (sl) # + np.sum(speed_left[1:3]))/4
    speed_right = np.roll(speed_right, 1)
    speed_right[0] = (sr) # + np.sum(speed_right[1:3]))/4
    ax1.clear()
    ax2.clear()
    ax1.plot(x_axis, speed_left, 'r-')
    ax1.plot(x_axis, speed_right, 'k-')
    plt.draw()

    # updare cx_neurons
    velocity = np.array([sl, sr])
    tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=0, velocity=velocity, tb1=tb1, memory=memory, cx=cx)
    angle, distance = cx.decode_cpu4(cpu4)
    #print((angle/np.pi) * 180, distance)

    # show frames
    cv2.imshow('vedio', cv2.resize(draw_flow(next, flow), (0,0), fx=1.0, fy=1.0))
    #print('Frame number: ', frame_num)
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break
    prvs = next
    start_time = time.time()
cap.release()
cv2.destroyAllWindows()





