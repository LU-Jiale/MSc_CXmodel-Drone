import dronekit
import socket
import exceptions
import time
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from CX_model.drone_basic import arm, arm_and_takeoff, download_mission, adds_square_mission, \
     PX4setMode, goto_position_target_local_ned
from pymavlink import mavutil

MAV_MODE_FLAG_GUIDED_ENABLED = 8 
MAV_MODE_FLAG_STABILIZE_ENABLED = 16
MAV_MODE_GUIDED_DISARMED = 88
MAV_MODE_STABILIZE_DISARMED = 80

# Try to connect to PX4
try:
    vehicle = dronekit.connect('/dev/ttyAMA0', baud=921600, wait_ready=True)
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
print "\nGet all vehicle attribute values:"
print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print " Local Location: %s" % vehicle.location.local_frame
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
print " GPS: %s" % vehicle.gps_0.fix_type
print " Battery: %s" % vehicle.battery
print " EKF OK?: %s" % vehicle._ekf_predposhorizabs
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Heading: %s" % vehicle.heading
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Groundspeed: %s" % vehicle.groundspeed    # settable
print " Airspeed: %s" % vehicle.airspeed    # settable
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable

if not vehicle:
    raise Exception("Fail to connect PX4")

vehicle.mode = VehicleMode("GUIDED")
time.sleep(1)
print vehicle.mode.name

while True:
    print "Range finder value:", vehicle.rangefinder.distance
    time.sleep(0.25)
vehicle.close()
