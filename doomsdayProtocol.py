#!/usr/bin/env python
# -*- coding: utf-8 -*-
import threading
import logging
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import serial as sl
from pymavlink import mavutil
import argparse  

vehicle = connect('udp:127.0.0.1:14551', wait_ready = True)

def arm_and_takeoff(aTargetAltitude):
    
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

#Gonna clear it first
def clear_mission():
    cmds = vehicle.commands
    vehicle.commands.clear()
    vehicle.flush()
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

clear_mission()

def getMission():
    print "Downloading mission"
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    missionList = []
    n_WP        = 0
    for wp in vehicle.commands:
        missionList.append(wp)
        n_WP += 1 
    print "Waypoint Count = %s" % n_WP
    return n_WP, missionList

def readmission(aFileName):
    print "Reading mission from file: %s\n" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

def uploadSurveyMission(aFileName):
        
        missionlist = readmission(aFileName)
        print "Uploading mission from that file : %s" % aFileName
        print "Clearing current mission..."
        clear_mission()
        cmds = vehicle.commands
        cmds.wait_ready()
        for command in missionlist:
            cmds.add(command)
        print "Uploading survey mission."
        cmds.upload()

def movetoTakingArea():
    cmds = vehicle.commands
    cmds.wait_ready()
    wpToArea = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           39.871990, 32.732210, 5)
    cmds.clear()
    cmds.add(wpToArea)
    cmds.upload()

def changeModeXL(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True

arm_and_takeoff(5)
vehicle.groundspeed = 15
movetoTakingArea()
vehicle.groundspeed = 1
vehicle.airspeed = 1
print "Current Mode : %s" % vehicle.mode
print "Changing it..."
changeModeXL(vehicle,"AUTO")
print "Current Mode : %s" % vehicle.mode
print "Moving to the pic-up area!"

print "Will wait for mission updates at Loiter mode."
changeModeXL(vehicle,"LOITER")
print "Mode changed to Loiter or we are fucked up IDK"

print "Uploading Survey mission."
uploadSurveyMission("gridPlainMission.txt")
print "Survey mission uploaded saksesfully from the fly."

print "Will change mode to Auto for mission to start!"
changeModeXL(vehicle,"AUTO")
print "Mode changed to Auto, hope that will work."

Found = False
while Found == False:
    #Lidar checks here
    print("Im checking the ground while my pilot rides...")
    if Found == True:
        changeModeXL(vehicle,"BRAKE")
        break

#What are you going to do after you find it...