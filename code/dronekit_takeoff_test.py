import dronekit
import socket
import exceptions
import time
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from CX_model.drone_ardupilot import arm, arm_and_takeoff, condition_yaw, send_ned_velocity
from pymavlink import mavutil

#connection_string = "127.0.0.1:14550"
connection_string = '/dev/ttyAMA0'

# Try to connect to PX4
try:
    vehicle = dronekit.connect(connection_string, baud=921600, wait_ready=True)
# Bad TCP connection
except socket.error:
    print 'No server exists!'
# Bad TTY connection
except exceptions.OSError as e:
    print 'No serial exists!'
# API Error
except dronekit.APIException:
    print 'Timeout!'
# Other error
except:
    print 'Some other error!'

# Get all vehicle attributes (state)
print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print " Local Location: %s" % vehicle.location.local_frame
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " EKF OK?: %s" % vehicle.ekf_ok
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Heading: %s" % vehicle.heading
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Groundspeed: %s" % vehicle.groundspeed    # settable
print " Airspeed: %s" % vehicle.airspeed    # settable
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable

char = raw_input("Takeoff mission, press anykey to continue, \'q\' to quit")
if char == 'q':
    raise Exception('Mission cancelled!')
else:
    print 'Mission start.'


arm_and_takeoff(vehicle, 5)
#time.sleep(10)

# movement
char = raw_input("Rotation completed, press anykey to continue moving mission, \'q\' to quit")
if char == 'q':
    raise Exception('Mission cancelled!')
else:
    print 'Movement start.'

count = 0
while vehicle.mode.name == "GUIDED" and count <5:
    print "moving"
    send_ned_velocity(vehicle, 1, 0, 0, 1)
    count += 1
    time.sleep(1)
count = 0
while vehicle.mode.name == "GUIDED" and count <5:
    print "moving"
    send_ned_velocity(vehicle, 0, 1, 0, 1)
    count += 1
    time.sleep(1)
send_ned_velocity(vehicle, 0, 0, 0, 1)

# rotation
char = raw_input("Takeoff completed, press anykey to continue rotation mission, \'q\' to quit")
if char == 'q':
    raise Exception('Mission cancelled!')
else:
    print 'Rotation start.'

vehicle.mode = VehicleMode("GUIDED")
time.sleep(1)
count = 0
while vehicle.mode.name == "GUIDED" and count <5:
    print "rotating"
    condition_yaw(vehicle, 10, relative=True)
    count += 1
    time.sleep(5)


vehicle.mode = VehicleMode('LAND')

vehicle.close()
