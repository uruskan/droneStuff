#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import threading
import logging
import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import math
import time
from pymavlink import mavutil
import serial as sl
import Magnet_control

#print('Connecting to vehicle on: %s' % connection_string)
#vehicle = connect(connection_string, wait_ready=True)
vehicle = connect('/dev/ttyACM0', wait_ready = True)
mag = Magnet_control.magnet()
vehicle.airspeed=3
vehicle.groundspeed=3
ay = 0
ax = 0
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

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


def alan_dolu(alt1, alt2):
    a1 = LocationGlobalRelative(39.8723430, 32.7323806, alt1)                   # Tam dolu alanı
    vehicle.simple_goto(a1, groundspeed=10)  # 39.8723430, 32.7323806
    time.sleep(20)
    a2 = LocationGlobalRelative(39.8723430, 32.7323806, alt2)
    vehicle.simple_goto(a2, groundspeed=10)
    time.sleep(15)

def alan_yarim(alt1, alt2):
    a1 = LocationGlobalRelative(39.8723286, 32.7320480, alt1)                     # Yarı dolu alanı
    vehicle.simple_goto(a1, groundspeed=10)  # 39.8723430, 32.7323806
    time.sleep(20)
    a2 = LocationGlobalRelative(39.8723286, 32.7320480, alt2)
    vehicle.simple_goto(a2, groundspeed=10)
    time.sleep(15)

class Sensor(threading.Thread):
    logging.basicConfig(level=logging.DEBUG,format='%(message)s', )

    def __init__(self):
        threading.Thread.__init__(self)
        return

    def run(self):
        ser = sl.Serial('/dev/ttyUSB0', baudrate=9600)
        while True:
            a1 = str(ser.readline())
            b = a1.split('/')
            logging.debug('Veriler: %s , %s' ,a1 , time.ctime(time.time()) )
            time.sleep(0.1)
            return b[0], b[1]

class Output(threading.Thread):

    logging.basicConfig(level=logging.DEBUG, format=' %(message)s')

    def __init__(self, ilk , son):
        threading.Thread.__init__(self)
        self.ilk = ilk
        self.son = son
        return

    def run(self):
        while True:
            logging.debug('Sonuc1: %s , sonuc2: %s ,  %s' , self.ilk , self.son, time.ctime(time.time()))
            return

class Velocity():
    def set_velocity_body(self,vehicle, vx, vy, vz):
        """ Remember: vz is positive downward!!!
        http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

        Bitmask to indicate which dimensions should be ignored by the vehicle
        (a value of 0b0000000000000000 or 0b0000001000000000 indicates that
        none of the setpoint dimensions should be ignored). Mapping:
        bit 1: x,  bit 2: y,  bit 3: z,
        bit 4: vx, bit 5: vy, bit 6: vz,
        bit 7: ax, bit 8: ay, bit 9:


        """
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  # -- BITMASK -> Consider only the velocities
            0, 0, 0,  # -- POSITION
            vx, vy, vz,  # -- VELOCITY
            0, 0, 0,  # -- ACCELERATIONS
            0, 0)
        
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.5)

def ironmaiden(x, y, w, h, frame, t, color, a):
    cv2.putText(frame, color + str(t), (int(x), int(y + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255))
    M = cv2.moments(a)
    ax = int(M["m10"] / M["m00"])
    ay = int(M["m01"] / M["m00"])
    cv2.circle(frame, (ax, ay), 3, (255, 255, 255), -1)
    cv2.putText(frame, "center", (ax - 20, ay - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return (ax, ay)


def moving(ax, ay):
    v2 = Velocity()
    if ax < 240:
        
        v2.set_velocity_body(vehicle, 1, 0, 0)
        print("ileri")
    elif ax > 240:
        v2.set_velocity_body(vehicle, -1, 0, 0)
        print("geri")
    if ay > 240:
        v2.set_velocity_body(vehicle, 0, -1, 0)
        print("sol")
    elif ay < 240:
        v2.set_velocity_body(vehicle, 0, 1, 0)
        print("sag")


arm_and_takeoff(5)
time.sleep(2)

print("Set default/target airspeed to 10")


clear_mission()
cmds = vehicle.commands
cmds.wait_ready()
firstArea = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           39.8720907, 32.7319237, 5)
cmds.add(firstArea())
cmds.upload()
time.sleep(1)
changeModeXL(vehicle,"AUTO")

################################################################

merkez = False
print("Kamera ciliyor")
camera = PiCamera()
camera.resolution = (480, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(480, 480))
lower_red = np.array([130, 150, 150])
upper_red = np.array([200, 255, 255])
time.sleep(0.1)
print ("Kamera acildi")

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    t = 0
    shot = frame.array
    cv2.circle(shot, (240, 240), 3, (255, 255, 255), -1)
    blur = cv2.GaussianBlur(shot, (11, 11), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    j = 0
    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernelOPen = np.ones((20, 20))
    mask = cv2.dilate(mask, kernelOPen, iterations=5)

    __, countours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if j == 0:
        for a in countours:
            peri = 0.01 * cv2.arcLength(a, True)
            approx = cv2.approxPolyDP(a, peri, True)
            if len(approx) >= 3:
                if (cv2.contourArea(a) > 100):
                    x, y, w, h = cv2.boundingRect(a)
                    color = "Fire"
                    ax, ay = ironmaiden(x, y, w, h, shot, t, color, a)
                    moving(ax, ay)


    cv2.imshow('cam', shot)
    cv2.imshow('mask', mask)
    rawCapture.truncate(0)
    time.sleep(8)
    if (ax == 0 and ay == 0):
        clear_mission()
        cmds = vehicle.commands
        cmds.wait_ready()
        firstArea = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           39.8720907, 32.7319237, 5)
        cmds.add(firstArea)
        cmds.upload()
        time.sleep(1)
        changeModeXL(vehicle,"AUTO")
    if ( 180<=ax<=300  and   180<=ay<=300):
        merkez = True
    if (merkez == True):                          
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon  # O an ki konum
        lowerAltitudeWP = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           lat, lon, 1.6)
        cmds.wait_ready()
        cmds.add(lowerAltitudeWP)
        cmds.upload()
        vehicle.changeModeXL(vehicle,"AUTO")
        time.sleep(1)
        camera.close()
        clear_mission()
        cv2.destroyAllWindows()
        break
    #####################################################################3
    
    
merkez = False
print("Kamera ciliyor")
camera = PiCamera()
camera.resolution = (480, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(480, 480))
lower_red = np.array([130, 150, 150])
upper_red = np.array([200, 255, 255])
time.sleep(0.1)
print ("Kamera acildi")

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    t = 0
    shot = frame.array
    cv2.circle(shot, (240, 240), 3, (255, 255, 255), -1)
    blur = cv2.GaussianBlur(shot, (11, 11), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    j = 0
    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernelOPen = np.ones((20, 20))
    mask = cv2.dilate(mask, kernelOPen, iterations=5)

    __, countours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if j == 0:
        for a in countours:
            peri = 0.01 * cv2.arcLength(a, True)
            approx = cv2.approxPolyDP(a, peri, True)
            if len(approx) >= 3:
                if (cv2.contourArea(a) > 100):
                    x, y, w, h = cv2.boundingRect(a)
                    color = "Fire"
                    ax, ay = ironmaiden(x, y, w, h, shot, t, color, a)
                    moving(ax, ay)


    cv2.imshow('cam', shot)
    cv2.imshow('mask', mask)
    rawCapture.truncate(0)
    if (ax == 0 and ay == 0):
        #1.6 metrede bulunamadi
        vehicle.groundspeed = 1
        vehicle.airspeed = 1
        changeModeXL(vehicle,"LOITER")
        uploadSurveyMission("gridPlainMission.txt")
        changeModeXL(vehicle,"AUTO")
    if ( 200<=ax<=280  and  200<=ay<=280):
        merkez = True
    if (merkez == True):
        mag.turnOn()                        
        lat1 = vehicle.location.global_relative_frame.lat
        lon1 = vehicle.location.global_relative_frame.lon  # O an ki konum
        lowerAltitudeWP1 = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           lat1, lon1, 0.35)
        cmds.wait_ready()
        cmds.add(lowerAltitudeWP1)
        cmds.upload()
        vehicle.changeModeXL(vehicle,"AUTO")
        time.sleep(4)
        clear_mission()
        riseWithPayload = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           lat1, lon1, 2)
        cmds.wait_ready()
        cmds.add(riseWithPayload)
        cmds.upload()
        vehicle.changeModeXL(vehicle,"AUTO")
        i1 = 0
        dataW = []
        while(i1 < 3):
            t = Sensor()
            ilk, son = t.run()
            th =  Output(ilk , son)
            th.start()
            weight = float((th3.weight).replace("\n", ""))
            dataW.append(weight)
            time.sleep(1)
            i1 = i1 + 1
        toplamW = dataW[0] + dataW[1] + dataW[2]
        ortalamaW = toplamW / 3
        fail = True
        if ortalamaW == 0:
            print ("Mission failed succesfully.")
        if ortalamaW != 0:
            fail = False
            camera.close()
        if fail == False:
            birakmaAlani = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                           39.8723430, 32.7323806, 5)
            cmds.wait_ready()
            cmds.add(birakmaAlani)
            cmds.upload()
            vehicle.changeModeXL(vehicle,"AUTO")
            time.sleep(10)
            mag.turnOff()
            print ("Payload birakildi.")
        
        
    ############################################################
    
    
"""
    
    i1 = 0
    dataW = []
    while(i1 < 3):
        t = Sensor()
        ilk, son = t.run()
        th =  Output(ilk , son)
        th.start()
        weight = float((th3.weight).replace("\n", ""))
        dataW.append(weight)
        time.sleep(1)
        i1 = i1 + 1
    toplamW = dataW[0] + dataW[1] + dataW[2]
    ortalamaW = toplamW / 3
        
    
    if(th.son == '0.00'):
        fail = True
            
    if (fail == True):       #basarisizlik durumunda#########################################################
        p = LocationGlobalRelative(39.8720907, 32.7319237, 5)
        go(p, 3)
        time.sleep(5)
        print("Kamera ciliyor")
        camera = PiCamera()
        camera.resolution = (480, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(480, 480))
        lower_red = np.array([130, 150, 150])
        upper_red = np.array([200, 255, 255])
        time.sleep(0.1)
        print ("Kamera acildi")

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            t = 0
            shot = frame.array
            cv2.circle(shot, (240, 240), 3, (255, 255, 255), -1)
            blur = cv2.GaussianBlur(shot, (11, 11), 0)
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            j = 0
            mask = cv2.inRange(hsv, lower_red, upper_red)
            kernelOPen = np.ones((20, 20))
            mask = cv2.dilate(mask, kernelOPen, iterations=5)

            __, countours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if j == 0:
                for a in countours:
                    peri = 0.01 * cv2.arcLength(a, True)
                    approx = cv2.approxPolyDP(a, peri, True)
                    if len(approx) >= 3:
                        if (cv2.contourArea(a) > 100):
                            x, y, w, h = cv2.boundingRect(a)
                            color = "Fire"
                            ax, ay = ironmaiden(x, y, w, h, shot, t, color, a)
                            moving(ax, ay)


            cv2.imshow('cam', shot)
            cv2.imshow('mask', mask)
            rawCapture.truncate(0)
            if ( 200<=ax<=280  and  200<=ay<=280):
                merkez = True
            if (merkez == True):                        
                lat1 = vehicle.location.global_relative_frame.lat
                lon1 = vehicle.location.global_relative_frame.lon  # O an ki konum
                currentLoc2 = LocationGlobalRelative(lat1, lon1, 0.50)
                vehicle.simple_goto(currentLoc2)
                time.sleep(3)
                camera.close()
                cv2.destroyAllWindows()
                merkez = False
                break
        
                #basarisizlik durumunda#########################################################
        

    if (fail == False):
        p_son = LocationGlobalRelative(lat, lon, 5)
        vehicle.simple_goto(p_son, 5)
        time.sleep(10)

        print("BASLADI")
        raw_data = []
        i2 = 0
        while i < 10:
            t2 = Sensor()
            ilk2 , son2 = t2.run()
            th2 = Output(ilk2, son2)
            th2.start()
            op = float((th3.weight).replace("\n", ""))
            raw_data.append(op)
            time.sleep(1)
            i2 = i2 + 1
        print(raw_data)
        toplam = raw_data[0] + raw_data[1] + raw_data[2] + raw_data[3] + raw_data[4] + raw_data[5] + raw_data[6] + \
                raw_data[7] + raw_data[8] + raw_data[9]
        ortalama = toplam / 10
        print(ortalama)

        if (ortalama > 600):
            alan_dolu(5, 1)
            mag.turnOff()
            alan_no = 1
            time.sleep(10)

        else:
            alan_yarim(5, 1)
            mag.turnOff()
            alan_no = 2
            time.sleep(10)

        print("Birinci sise bitti!!!")
        break
     ###############################################################
    
    
    
    ###########ikinci sise########################################
go() #ikinci sise konumunu ekle!!!!
    

merkez = False
print("Kamera ciliyor")
camera = PiCamera()
camera.resolution = (480, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(480, 480))
lower_red = np.array([130, 150, 150])
upper_red = np.array([200, 255, 255])
time.sleep(0.1)
print ("Kamera acildi")

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    t = 0
    shot = frame.array
    cv2.circle(shot, (240, 240), 3, (255, 255, 255), -1)
    blur = cv2.GaussianBlur(shot, (11, 11), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    j = 0
    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernelOPen = np.ones((20, 20))
    mask = cv2.dilate(mask, kernelOPen, iterations=5)

    __, countours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if j == 0:
        for a in countours:
            peri = 0.01 * cv2.arcLength(a, True)
            approx = cv2.approxPolyDP(a, peri, True)
            if len(approx) >= 3:
                if (cv2.contourArea(a) > 100):
                    x, y, w, h = cv2.boundingRect(a)
                    color = "Fire"
                    ax, ay = ironmaiden(x, y, w, h, shot, t, color, a)
                    moving(ax, ay)


    cv2.imshow('cam', shot)
    cv2.imshow('mask', mask)
    rawCapture.truncate(0)
    if ( 180<=ax<=300  and   180<=ay<=300):
        merkez = True
    if (merkez == True):                          
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon  # O an ki konum
        currentLoc = LocationGlobalRelative(lat, lon, 2.5)
        vehicle.simple_goto(currentLoc)
        time.sleep(4)
        camera.close()
        cv2.destroyAllWindows()
        break
    #####################################################################3
    
    
merkez = False
print("Kamera ciliyor")
camera = PiCamera()
camera.resolution = (480, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(480, 480))
lower_red = np.array([130, 150, 150])
upper_red = np.array([200, 255, 255])
time.sleep(0.1)
print ("Kamera acildi")

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    t = 0
    shot = frame.array
    cv2.circle(shot, (240, 240), 3, (255, 255, 255), -1)
    blur = cv2.GaussianBlur(shot, (11, 11), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    j = 0
    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernelOPen = np.ones((20, 20))
    mask = cv2.dilate(mask, kernelOPen, iterations=5)

    __, countours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if j == 0:
        for a in countours:
            peri = 0.01 * cv2.arcLength(a, True)
            approx = cv2.approxPolyDP(a, peri, True)
            if len(approx) >= 3:
                if (cv2.contourArea(a) > 100):
                    x, y, w, h = cv2.boundingRect(a)
                    color = "Fire"
                    ax, ay = ironmaiden(x, y, w, h, shot, t, color, a)
                    moving(ax, ay)


    cv2.imshow('cam', shot)
    cv2.imshow('mask', mask)
    rawCapture.truncate(0)
    if ( 200<=ax<=280  and  200<=ay<=280):
        merkez = True
    if (merkez == True):                        
        lat1 = vehicle.location.global_relative_frame.lat
        lon1 = vehicle.location.global_relative_frame.lon  # O an ki konum
        currentLoc2 = LocationGlobalRelative(lat1, lon1, 0.50)
        mag.turnOn()
        vehicle.simple_goto(currentLoc2)
        time.sleep(3)
        camera.close()
        cv2.destroyAllWindows()
        break
    ############################################################
    
    ###########ikinci while########################################
    
fail = False
while True:
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon  # O an ki konum
    time.sleep(10)
    
    ps = LocationGlobalRelative(lat, lon, 1.25)
    vehicle.simple_goto(ps, 3)
    
    i1 = 0
    dataW = []
    while(i1 < 3):
        t = Sensor()
        ilk, son = t.run()
        th =  Output(ilk , son)
        th.start()
        weight = float((th3.weight).replace("\n", ""))
        dataW.append(weight)
        time.sleep(1)
        i1 = i1 + 1
    toplamW = dataW[0] + dataW[1] + dataW[2]
    ortalamaW = toplamW / 3
        
    
    if(th.son == '0.00'):
        fail = True
            
    if (fail == True):       #basarisizlik durumunda#########################################################
        p = LocationGlobalRelative(39.8720907, 32.7319237, 5)
        go(p, 3)
        time.sleep(5)
        print("Kamera ciliyor")
        camera = PiCamera()
        camera.resolution = (480, 480)
        camera.framerate = 32
        rawCapture = PiRGBArray(camera, size=(480, 480))
        lower_red = np.array([130, 150, 150])
        upper_red = np.array([200, 255, 255])
        time.sleep(0.1)
        print ("Kamera acildi")

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            t = 0
            shot = frame.array
            cv2.circle(shot, (240, 240), 3, (255, 255, 255), -1)
            blur = cv2.GaussianBlur(shot, (11, 11), 0)
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            j = 0
            mask = cv2.inRange(hsv, lower_red, upper_red)
            kernelOPen = np.ones((20, 20))
            mask = cv2.dilate(mask, kernelOPen, iterations=5)

            __, countours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if j == 0:
                for a in countours:
                    peri = 0.01 * cv2.arcLength(a, True)
                    approx = cv2.approxPolyDP(a, peri, True)
                    if len(approx) >= 3:
                        if (cv2.contourArea(a) > 100):
                            x, y, w, h = cv2.boundingRect(a)
                            color = "Fire"
                            ax, ay = ironmaiden(x, y, w, h, shot, t, color, a)
                            moving(ax, ay)


            cv2.imshow('cam', shot)
            cv2.imshow('mask', mask)
            rawCapture.truncate(0)
            if ( 200<=ax<=280  and  200<=ay<=280):
                merkez = True
            if (merkez == True):                        
                lat1 = vehicle.location.global_relative_frame.lat
                lon1 = vehicle.location.global_relative_frame.lon  # O an ki konum
                currentLoc2 = LocationGlobalRelative(lat1, lon1, 0.50)
                vehicle.simple_goto(currentLoc2)
                time.sleep(3)
                camera.close()
                cv2.destroyAllWindows()
                merkez = False
                break
        
                #basarisizlik durumunda#########################################################
        

    if (fail == False):
        p_son = LocationGlobalRelative(lat, lon, 5)
        vehicle.simple_goto(p_son, 5)
        time.sleep(10)

        if (alan_no == 1):
            alan_yarim(5, 1)
            mag.turnOff()
            time.sleep(10)

        else:
            alan_dolu(5, 1)
            mag.turnOff()
            time.sleep(10)

        print("RTL modu")
        break
    
    ###########ikinci while########################################

vehicle.mode = VehicleMode("RTL")
"""