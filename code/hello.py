import dronekit
import socket
import exceptions
import time
from dronekit import VehicleMode
from drone_basic import arm, arm_and_takeoff, download_mission

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

time.sleep(15)

if vehicle:
    print('Mode:', vehicle.mode.name)
    cmds = download_mission(vehicle)
    print ('Waypoint numbers: ', cmds.count)
    missionlist=[]
    for cmd in cmds:
        missionlist.append(cmd)
        print(cmd.command)

    arm_and_takeoff(vehicle, 5)
    time.sleep(10)
    vehicle.mode = VehicleMode('LAND')
''' for i in range(20):
        if vehicle.mode == VehicleMode("OFFBOARD"):
            print('Heading:', vehicle.heading)
            print('Velocity:', vehicle.velocity)
            time.sleep(0.5)
            vehicle.armed   = True
            time.sleep(2)
            vehicle.armed   = False
        else:
            print 'Interrupt from controller, mission is stopped.'
            break;
'''
vehicle.close()
