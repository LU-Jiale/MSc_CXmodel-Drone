import numpy as np
import sys, os, time
import logging, datetime
import cx_rate, cx_basic
import central_complex
import dronekit
from central_complex import update_cells
from drone_basic import arm, arm_and_takeoff

# initialize logger
time_string = str(datetime.datetime.now()).replace(':', '-').replace(' ', '_').split('.')[0]
fname = 'log/gps_' + time_string + '.log'
logging.basicConfig(filename=fname,level=logging.DEBUG)

# initialize CX models

cx_gps = cx_rate.CXRate(noise = 0)
MAV_CMD_NAV_LANDtb1_gps = np.zeros(central_complex.N_TB1)
memory_gps = 0.5 * np.ones(central_complex.N_CPU4)
cpu4_gps = np.zeros(16)

# connect to PX4 and arm
try:
    drone = dronekit.connect('/dev/ttyAMA0', baud = 921600, heartbeat_timeout=15)
except dronekit.APIException:
    logging.critical('Timeout! Fail to connect PX4')
    raise Exception('Timeout! Fail to connct PX4')
except:
    logging.critical('Some other error!')
    raise Exception('Fail to connct PX4')

while drone.mode.name != "RTL":
    
    # update CX neurons
    drone_heading = drone.heading/180*np.pi

    velocity = drone.velocity
    if velocity[0]:
        left_real = (velocity[0]*np.cos(drone_heading-np.pi/4) + \
                     velocity[1]*np.cos(drone_heading-np.pi/4-np.pi/2))
        right_real = (velocity[0]*np.cos(drone_heading+np.pi/4) + \
                      velocity[1]*np.cos(drone_heading+np.pi/4-np.pi/2))
        velocity = np.array([left_real, right_real])
        __, __, tb1_gps, __, __, memory_gps, cpu4_gps, __, motor_gps = \
                update_cells(heading=drone_heading, velocity=velocity, \
                             tb1=tb1_gps, memory=memory_gps, cx=cx_gps)
    else:
        raise Exception("No GPS signal.")
   
    # logging
    logging.info('sl:{} sr:{} heading:{} velocity:{} position:{}'.format(
                0,0,drone.heading,drone.velocity, drone.location.global_relative_frame))
    angle_gps, distance_gps = cx_gps.decode_cpu4(cpu4_gps)
    logging.info('Angle_optical:{} Distance_optical:{} Angle_gps:{} Distance_gps:{} \
                 elapsed_time:{}'.format(0, 0, (angle_gps/np.pi)*180, distance_gps, elapsed_time))

    prvs = next
#    print('Elapsed time:%.5f'%elapsed_time)

print "Mission ended or stoppped. The final results of CX model based on optcial flow is:"
print((angle_gps/np.pi) * 180, distance_gps)
drone.close()

