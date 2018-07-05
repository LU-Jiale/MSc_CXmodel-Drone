import numpy as np
import cv2
import central_complex
import cx_rate
import cx_basic
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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
cap = cv2.VideoCapture('sample2.avi')
cap.set(cv2.CAP_PROP_FRAME_WIDTH,200)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,130)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fw_quarter = int(fw/4)
print("Frame size: {0}*{1}".format(fw,fh))

# filter for speed retrieval
left_filter = np.zeros([fh,fw,2])
right_filter = np.zeros([fh,fw,2])
left_filter[:,:,0] = np.cos(3*np.pi/4)
left_filter[:,:,1] = np.sin(3*np.pi/4)
right_filter[:,:,0] = np.cos(np.pi/4)
right_filter[:,:,1] = np.sin(np.pi/4)
# visulize computed speed 
plt.ion()

fig, (ax1, ax2) = plt.subplots(2, sharey=True)
ax1.set(title='speed', ylabel='left')
ax2.set(xlabel='time (s)', ylabel='right')
speed_left = np.zeros(300)
speed_right = np.zeros(300)
row = np.arange(300)
plt.show()

ret, frame1 = cap.read()
prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[...,1] = 255


while(1):
    start_time = time.time()
    # Image processing, compute optical flow
    ret, frame2 = cap.read()
    next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    mag = np.sqrt(np.power(flow[:,:,0],2) + np.power(flow[:,:,1],2))
    elapsed_time = time.time() - start_time
    mag = mag.flatten()
    count = [i for i in mag if i >= 1]
    N = len(count)
    weight = 10000/(fw*fh)
    sl = np.sum(flow * left_filter)/(N+10000)
    sr = np.sum(flow * right_filter)/(N+10000)

    # visulize computed speed 
    speed_left = np.roll(speed_left, 1)
    speed_left[0] = sl
    speed_right = np.roll(speed_right, 1)
    speed_right[0] = sr
    ax1.clear()
    ax2.clear()
    ax1.plot(row, speed_left, 'k-')
    ax2.plot(row, speed_right, 'k-')
    plt.draw()
    print(len(count))
    # show video
    cv2.imshow('vedio', frame2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    
    prvs = next
cap.release()
cv2.destroyAllWindows()





