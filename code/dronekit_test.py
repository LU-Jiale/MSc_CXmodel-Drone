import dronekit
import socket
import exceptions
import time
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from CX_model.drone_basic import arm, arm_and_takeoff, download_mission, adds_square_mission
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
print "   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target
print "   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned
print "   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int
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

print 'Check the status, press \'g\' to continue, \'q\' to quit'
char = cv2.waitKey()
if char & 0xFF == ord('q'):
    return 0
elif char & 0xFF == ord('g'):
    print 'Mission start.'

if vehicle:
    # old mission
    cmds = download_mission(vehicle)
    print ('Waypoint numbers: ', cmds.count)
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)
        print(cmd.x, cmd.y, cmd.z)
    
    time.sleep(10)
    # modify mission
    cmd = missionlist[1]
    startlocation=LocationGlobal(cmd.x, cmd.y,cmd.z)
    adds_square_mission(vehicle, startlocation, 50)
    time.sleep(3)

    # new mission
    cmds = download_mission(vehicle)
    print ('Waypoint numbers: ', cmds.count)
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)
        print(cmd.x, cmd.y, cmd.z)

#    arm_and_takeoff(vehicle, 5)
    time.sleep(10)
    vehicle.mode = VehicleMode('GUIDED')
    '''
    print 'Create a new mission (for current location)'
    adds_square_mission(vehicle.location.global_frame,50)


    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    arm_and_takeoff(5)

    print "Starting mission"
    # Reset mission set to first (0) waypoint
    vehicle.commands.next=0

    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("MISSION")


    # Monitor mission. 
    # Demonstrates getting and setting the command number 
    # Uses distance_to_current_waypoint(), a convenience function for finding the 
    #   distance to the next waypoint.

    while vehicle.mode == VehicleMode('MISSION'):
        nextwaypoint=vehicle.commands.next
        print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
  
        if nextwaypoint==3: #Skip to next waypoint
            print 'Skipping to Waypoint 5 when reach waypoint 3'
            vehicle.commands.next = 5
        if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print "Exit 'standard' mission when start heading to final waypoint (5)"
            break;
        time.sleep(1)

    print 'Return to launch'
    vehicle.mode = VehicleMode("RTL")
    '''

vehicle.close()
