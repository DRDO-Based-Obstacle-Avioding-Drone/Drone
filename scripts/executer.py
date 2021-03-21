#!/usr/bin/env python

"""
set_attitude_target.py: (Copter Only)
This example shows how to move/direct Copter and send commands
 in GUIDED_NOGPS mode using DroneKit Python.
Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
        Always watch the drone movement, and make sure that you are in dangerless environment.
        Land the drone as soon as possible when it shows any unexpected behavior.
Tested in Python 2.7.10
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import rospy
from std_msgs.msg import Float32
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default= 'udp:127.0.0.1:14550')
args = parser.parse_args()
# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)
rospy.init_node('attitude_controller')
hght = 3
latd = -35.3632591
longd =  149.1653452


def lat_to_x(input_latitude):
    return 110692.0702932625*(input_latitude+35.3632591)
def long_to_x(input_longitude):
    return -105292.0089353767*(input_longitude-149.1653452)
def x_to_lat(input_x):
	return input_x/110692.0702932625 - 35.3632591
def x_to_long(input_x):
	return input_x/(-105292.0089353767) + 149.1653452


def demo(data):
    global hght
    hght = data.data
   

def demo2(data):
	global latd
	latd = x_to_lat(data.data)
	# print("\nsubscribedlat->",data.data, '   ', latd,'\n\n')
	
    

def demo3(data):
	global longd
	longd = x_to_long(data.data)
	# print("\nsubscribedlong->",data.data, '   ', longd,'\n\n')
    

rospy.Subscriber('/Thrust', Float32, demo)
rospy.Subscriber('/Roll', Float32, demo2)
rospy.Subscriber('/Pitch', Float32, demo3)



# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

	print "Basic pre-arm checks"
	# Don't let the user try to arm until autopilot is ready
	while not vehicle.is_armable:
		print " Waiting for vehicle to initialise..."
		time.sleep(1)

	print "Arming motors"
	# Copter should arm in GUIDED mode
	vehicle.mode    = VehicleMode("GUIDED")
	vehicle.armed   = True

	while not vehicle.armed:
		print " Waiting for arming..."
		time.sleep(1)

	print "Taking off!"
	vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

	# Check that vehicle has reached takeoff altitude
	while True:
		print " Altitude: ", vehicle.location.global_relative_frame.alt 
		#Break and return from function just below target altitude.        
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
			print "Reached target altitude"
			break
		time.sleep(1)


if __name__ == '__main__':
#     vehicle.mode = VehicleMode("LAND")

# # Close vehicle object
# vehicle.close()
    arm_and_takeoff(3)
    print("Setting airspeed")
    vehicle.airspeed = 3
    cur_lat = 0
    cur_long = 0
    cur_hgt = 0
    while True:
        print((latd),lat_to_x(latd),(longd),long_to_x(longd),hght)
        if cur_lat!=latd or cur_long!=longd or cur_hgt!=hght:
            point1 = LocationGlobalRelative(latd, longd, hght)
            vehicle.simple_goto(point1)
            cur_lat = latd
            cur_long = longd
            cur_hgt = hght
        time.sleep(0.1)

        

	# print("Going towards first point for 30 seconds ...")
	# point1 = LocationGlobalRelative(-35.3632591, 149.1653452,10)
	# vehicle.simple_goto(point1)
	# print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
	# print("\n\n\nhello i am here here here ",vehicle.location.global_relative_frame.alt,"\n\n\n\n\n")
	
	# print("\n\n\nhello i am here here here ",vehicle.location.global_relative_frame.alt,"\n\n\n\n\n")