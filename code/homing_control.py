import numpy as np
import sys, os, time
import logging, datetime
import dronekit
import argparse
import cv2
from CX_model import cx_rate, central_complex
from dronekit import VehicleMode
from CX_model.optical_flow import Optical_flow, FRAME_DIM
from CX_model.central_complex import update_cells
from CX_model.drone_ardupilot import arm, arm_and_takeoff

connection_string = "127.0.0.1:14550"

# initialize logger
time_string = str(datetime.datetime.now()).replace(':', '-').replace(' ', '_').split('.')[0]
fname = 'sim_log/' + time_string + '.log'
logging.basicConfig(filename=fname,level=logging.DEBUG)

# initialize CX models
cx_gps = cx_rate.CXRate(noise = 0)
tb1_gps = np.zeros(central_complex.N_TB1)
memory_gps = 0.5 * np.ones(central_complex.N_CPU4)
cpu4_gps = np.zeros(16)

# connect to PX4 and arm
try:
    drone = dronekit.connect(connection_string, baud = 51600, heartbeat_timeout=15)
except dronekit.APIException:
    logging.critical('Timeout! Fail to connect PX4')
    raise Exception('Timeout! Fail to connct PX4')
except:
    logging.critical('Some other error!')
    raise Exception('Fail to connct PX4')
state = arm_and_takeoff(drone, 5)

# set to mission mode.
drone.mode = VehicleMode("AUTO")
while drone.mode.name != "AUTO":
    print "Pre-intialisation, waiting for the AUTO mode."
    time.sleep(2)
# wait until reach first waypoint, 1->home, 2->takeoff
nextwaypoint = drone.commands.next
while nextwaypoint <= 1:
    print "Initialisation, Moving to waypoint", drone.commands.next+1
    nextwaypoint = drone.commands.next
    time.sleep(1)

sl = 0
sr = 0
angle_optical = 0
distance_optical = 0

start_time = time.time()
print "Start to update CX model, switch mode to end"
while drone.mode.name == "AUTO":
    
    # update CX neurons
    drone_heading = drone.heading/180.0*np.pi
    velocity = drone.velocity
    if velocity[0]:
        left_real = (velocity[0]*np.cos(drone_heading-np.pi/4) + \
                     velocity[1]*np.cos(drone_heading-np.pi/4-np.pi/2))
        right_real = (velocity[0]*np.cos(drone_heading+np.pi/4) + \
                      velocity[1]*np.cos(drone_heading+np.pi/4-np.pi/2))
        velocity_gps = np.array([left_real, right_real]) / 4.0   # normarlize velocity [-1,1]
        __, __, tb1_gps, __, __, memory_gps, cpu4_gps, __, motor_gps = \
                update_cells(heading=drone_heading, velocity=velocity_gps, \
                             tb1=tb1_gps, memory=memory_gps, cx=cx_gps)

    elapsed_time = time.time() - start_time
    start_time = time.time()
    # logging
    logging.info('sl:{} sr:{} heading:{} velocity:{} position:{}'.format(
                sl,sr,drone.heading,drone.velocity, drone.location.global_relative_frame))
    angle_gps, distance_gps = cx_gps.decode_cpu4(cpu4_gps)
    logging.info('Angle_optical:{} Distance_optical:{} Angle_gps:{} Distance_gps:{} \
                 elapsed_time:{}'.format((angle_optical/np.pi)*180.0, distance_optical, \
                 (angle_gps/np.pi)*180.0, distance_gps, elapsed_time))

    # moniter the mission
    if frame_num%10==0:
        print('heading:{} Angle:{} Distance:{} motor:{}'.format(drone.heading, 
              (angle_gps/np.pi)*180.0, distance_gps, motor_gps))

        display_seq = drone.commands.next
        print "Moving to waypoint: ", display_seq
        print('Elapsed time:%.5f'%elapsed_time)
        nextwaypoint = drone.commands.next

    time.sleep(0.1)

print "Mission ended or stoppped. The final results of CX model based on optcial flow is:"
print((angle_optical/np.pi) * 180, distance_optical)
drone.close()
cv2.destroyAllWindows()
