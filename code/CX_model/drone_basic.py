import time
import dronekit
from dronekit import VehicleMode

'''Basic control functions for drone(F450)

  ('Available modes: ', ['ALTCTL', 'STABILIZED', 'OFFBOARD', 'LAND', 'POSCTL', 'RTL', 
    'MANUAL', 'MISSION', 'RATTITUDE', 'RTGS', 'LOITER', 'ACRO', 'FOLLOWME'])
  indoor enviroment: use ALTCTL for fixed height flight
  oudoor enviroment(GPS avalid): use position mode

'''

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    # Basic pre-arm checks
    if not vehicle.is_armable:
        for i in range(6):
            print " Waiting for vehicle to initialise..."
    	    if vehicle.is_armable:
                break
            time.sleep(5)    
        if not vehicle.is_armable:
            print "Initialisation failed! Mission cancelled."
            return "Initialisation failed! Mission cancelled."

    # Arming motors after checking the mode is set up properly:
    vehicle.mode = VehicleMode("POSCTL")
    time.sleep(5)
    if not vehicle.mode == VehicleMode("POSCTL"):
        
        print "Mode setup failed! Takeoff cancelled."
        return "Mode setup failed! Takeoff cancelled."
    else:
        vehicle.armed = True

    # Confirm vehicle armed before attempting to take off   
    if not vehicle.armed:
        for i in range(6):
            print " Waiting for vehicle to arm..."
    	    if vehicle.armed:
                break
            time.sleep(5)    
        if not vehicle.armed:
            print "Arm motors failed! Mission cancelled."
            return "Arm motors failed! Mission cancelled."

    # Taking off
    #vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        if vehicle.mode == VehicleMode("POSCTL"):
            print " Altitude: ", vehicle.location.global_relative_frame.alt
            #Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print "Reached target altitude"
                break
            time.sleep(1)
        else:
            print 'Interrupt from controller, Mission cancelled.'
            return 'Interrupt from controller, Mission cancelled.'
    return "Take off finished"

def arm(vehicle):
    """
    Arms vehicle 
    """

    # Basic pre-arm checks
    if not vehicle.is_armable:
        for i in range(6):
            print " Waiting for vehicle to initialise..."
    	    if vehicle.is_armable:
                break
            time.sleep(5)
        if not vehicle.is_armable:
            print "Initialisation failed! Mission cancelled."
            return "Initialisation failed! Mission cancelled."

    # Arming motors after checking the mode is set up properly:
    vehicle.mode = VehicleMode("POSCTL")
    time.sleep(5)
    if not vehicle.mode == VehicleMode("POSCTL"):
        print "Mode setup failed! Takeoff cancelled."
        return "Mode setup failed! Takeoff cancelled."
    else:
        vehicle.armed = True

    # Confirm vehicle armed before attempting to do other thing
    if not vehicle.armed:
        for i in range(6):
            print " Waiting for vehicle to arm..."
    	    if vehicle.armed:
                break
            time.sleep(5)
        if not vehicle.armed:
            print "Arm motors failed! Mission cancelled."
            return "Arm motors failed! Mission cancelled."

    return "Armed successfully"

