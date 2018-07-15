import dronekit
import socket
import exceptions
import time
from dronekit import VehicleMode

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    ('Available modes: ', ['ALTCTL', 'STABILIZED', 'OFFBOARD', 'LAND', 'POSCTL', 'RTL', 
    'MANUAL', 'MISSION', 'RATTITUDE', 'RTGS', 'LOITER', 'ACRO', 'FOLLOWME'])
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDVehicleMode("GUIDED")ED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    #print "Taking off!"
    #vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

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

if vehicle:
    print('Mode:', vehicle.mode.name)
    vehicle.mode = VehicleMode("ALTCTL")
#    arm_and_takeoff(vehicle, 20)
    for i in range(20):
        if vehicle.mode == VehicleMode("ALTCTL"):
            print('Heading:', vehicle.heading)
            print('Velocity:', vehicle.velocity)
            time.sleep(0.5)
            vehicle.armed   = True
            time.sleep(2)
            vehicle.armed   = False
        else:
            print 'Interrupt from controller, mission is stopped.'
            break;
vehicle.close()
