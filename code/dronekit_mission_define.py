import dronekit
import socket
import exceptions
import time
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from CX_model.drone_basic import arm, arm_and_takeoff, download_mission, adds_square_mission, PX4setMode
from pymavlink import mavutil

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
print " Battery: %s" % vehicle.battery
print " Heading: %s" % vehicle.heading
print " System status: %s" % vehicle.system_status.state
print " Groundspeed: %s" % vehicle.groundspeed    # settable
print " Airspeed: %s" % vehicle.airspeed    # settable
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable

char = raw_input("Check the status, press anykey to continue, \'q\' to quit")
if char == 'q':
    raise Exception('Mission cancelled!')
else:
    print 'Mission start.'

if vehicle:
    #Callback to print the location in global frames. 'value' is the updated value
    def location_callback(self, attr_name, value):
    print "Location (Global): ", value


    # Add a callback `location_callback` for the `global_frame` attribute.
    vehicle.add_attribute_listener('location.global_frame', location_callback)

    # Load commands
    cmds = vehicle.commands
    cmds.clear()

    home = vehicle.location.global_relative_frame

    # takeoff to 10 meters
    wp = get_location_offset_meters(home, 0, 0, 10);
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                  mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    
    
    # land
    wp = get_location_offset_meters(home, 0, 0, 10);
    cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                  mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
     cmds.add(cmd)

    # Upload mission
    cmds.upload()
    time.sleep(2)

    vehicle.armed = True
    # monitor mission execution
    nextwaypoint = vehicle.commands.next
    while nextwaypoint < len(vehicle.commands):
        if vehicle.commands.next > nextwaypoint:
            display_seq = vehicle.commands.next+1
            print "Moving to waypoint %s" % display_seq
            nextwaypoint = vehicle.commands.next
        time.sleep(1)

    # wait for the vehicle to land
    while vehicle.commands.next > 0:
        time.sleep(1)


    # Disarm vehicle
    vehicle.armed = False
    time.sleep(1)

    # Close vehicle object before exiting script
    vehicle.close()

time.sleep(1)

