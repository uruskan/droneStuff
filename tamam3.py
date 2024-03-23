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
#print('Connecting to vehicle on: %s' % connection_string)
#vehicle = connect(connection_string, wait_ready=True)
vehicle = connect('/dev/ttyACM0', wait_ready = True)
vehicle.airspeed=3
vehicle.groundspeed=3
yarimAlan = LocationGlobalRelative(39.872240,32.732374,5)
doluAlan = LocationGlobalRelative(39.872235,32.732020,5)
nokta1 = LocationGlobalRelative()
nokta2 = LocationGlobalRelative()
nokta3 = LocationGlobalRelative()
nokta4 =  LocationGlobalRelative()
nokta5 = LocationGlobalRelative()
ay = 0
ax = 0
monkeyKing = Magnet_control.magnet()


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


def go(p):
    vehicle.simple_goto(p)


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
        time.sleep(0.1)

def ironmaiden(x, y, w, h, frame, t, color, a):
    cv2.putText(frame, color + str(t), (int(x), int(y + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255))
    M = cv2.moments(a)
    ax = int(M["m10"] / M["m00"])
    ay = int(M["m01"] / M["m00"])
    cv2.circle(frame, (ax, ay), 3, (255, 255, 255), -1)
    cv2.putText(frame, "center", (ax - 20, ay - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    print ax,ay
    return (ax, ay)


def moving(ax, ay):
    v2 = Velocity()
    if ax < 240:
        v2.set_velocity_body(vehicle, 0.6, 0, 0)
        print("ileri")
    elif ax > 240:
        v2.set_velocity_body(vehicle, -0.6, 0, 0)
        print("geri")
    if ay > 240:
        v2.set_velocity_body(vehicle, 0, -0.6, 0)
        print("sol")
    elif ay < 240:
        v2.set_velocity_body(vehicle, 0, 0.6, 0)
        print("sag")


arm_and_takeoff(5)
time.sleep(1)
vehicle.simple_goto(nokta1)
time.sleep(8)
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
    time.sleep(6) #Bi bekleyelim goruyor muyuz diye
    if ax == 0 and ay == 0: #Gormuyoruz bu yuzden gorebilecegimiz bir noktaya gidelim
             #Buraya koordinatlar gelecek.
            vehicle.simple_goto(nokta2,0.5)
            time.sleep(5)
            camera.close()
            ###############################################################

            merkez1 = False
            print("ikinci kez kamerayi aciyorum")
            camera = PiCamera()
            camera.resolution = (480, 480)
            camera.framerate = 32
            rawCapture = PiRGBArray(camera, size=(480, 480))
            lower_red = np.array([130, 150, 150])
            upper_red = np.array([200, 255, 255])
            time.sleep(0.1)
            print ("Acildi")

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
                time.sleep(6)
                if (ax == 0 and ay == 0):
                    
                    
                    vehicle.simple_goto(nokta3,0.5)
                    time.sleep(5)
                    camera.close()
                    
                    ###############################################################

                    merkez3 = False
                    print("Ucuncu kez kamera ciliyor")
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
                            
                            
                            vehicle.simple_goto(nokta4,0.5)
                            time.sleep(5)
                            camera.close()

                            ##################################################################

                            ###############################################################

                            merkez4 = False
                            print("Kamera dorduncu kez ciliyor")
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
                                    
                                    vehicle.simple_goto(nokta5,0.5)
                                    time.sleep(5)
                                    camera.close()

                                    merkez5 = False
                                    print("Kamera besinci kez ciliyor")
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
                                        if ax == 0 and ay == 0:
                                            print "Sen uçma knk"
                                            vehicle.mode = VehicleMode("RTL")
                                        if ( 180<=ax<=300  and   180<=ay<=300):
                                            merkez5 = True
                                        if (merkez5 == True):
                                            monkeyKing.turnOn()                        
                                            lat = vehicle.location.global_relative_frame.lat
                                            lon = vehicle.location.global_relative_frame.lon  # O an ki konum
                                            currentLoc5 = LocationGlobalRelative(lat, lon, 1.25)
                                            vehicle.simple_goto(currentLoc5)
                                            time.sleep(6)
                                            currentLoc5High = LocationGlobalRelative(lat,lon,1.25)
                                            vehicle.simple_goto(currentLoc5High)
                                            """
                                            Normalde burada ağırlık ölçüm vs. yapmalıydı
                                            """
                                            time.sleep(5)
                                            vehicle.simple_goto(doluAlan)
                                            time.sleep(10)
                                            monkeyKing.turnOff()
                                            camera.close()
                                            cv2.destroyAllWindows()
                                            break
                                    
                                if ( 180<=ax<=300  and   180<=ay<=300):
                                    merkez4 = True
                                if (merkez4 == True):
                                    
                                    monkeyKing.turnOn()                          
                                    lat = vehicle.location.global_relative_frame.lat
                                    lon = vehicle.location.global_relative_frame.lon  # O an ki konum
                                    currentLoc4 = LocationGlobalRelative(lat, lon, 1.25)
                                    vehicle.simple_goto(currentLoc4)
                                    time.sleep(6)
                                    camera.close()
                                    currentLoc4High = LocationGlobalRelative(lat,lon,5)
                                    vehicle.simple_goto(currentLoc4High)
                                    time.sleep(6)
                                    """
                                    Burada normalde kalkip siseyi aldigini vs. kontrol etmesi ve ona gore alanlara ilerlemesi lazim
                                    """
                                    vehicle.simple_goto(yarimAlan)
                                    time.sleep(10)
                                    monkeyKing.turnOff()
                                    cv2.destroyAllWindows()
                                    break
                                #####################################################################3

                        if ( 180<=ax<=300  and   180<=ay<=300):
                            merkez3 = True
                        if (merkez3 == True):                          
                            
                            monkeyKing.turnOn()
                            lat = vehicle.location.global_relative_frame.lat
                            lon = vehicle.location.global_relative_frame.lon  # O an ki konum
                            currentLoc3 = LocationGlobalRelative(lat, lon, 1.25)
                            vehicle.simple_goto(currentLoc3)
                            time.sleep(6)
                            camera.close()
                            currentLoc3High = LocationGlobalRelative(lat,lon,5)
                            vehicle.simple_goto(currentLoc3High)
                            time.sleep(6)
                            """
                            Burada normalde kalkip siseyi aldigini vs. kontrol etmesi ve ona gore alanlara ilerlemesi lazim
                            """
                            vehicle.simple_goto(doluAlan)
                            time.sleep(10)
                            monkeyKing.turnOff()
                            cv2.destroyAllWindows()
                            break
    #####################################################################3

                if ( 180<=ax<=300  and   180<=ay<=300):
                    merkez2 = True

                if (merkez2 == True):                          
                    monkeyKing.turnOn()
                    lat = vehicle.location.global_relative_frame.lat
                    lon = vehicle.location.global_relative_frame.lon  # O an ki konum
                    currentLoc2 = LocationGlobalRelative(lat, lon, 1.25)
                    vehicle.simple_goto(currentLoc2)
                    time.sleep(6)
                    camera.close()
                    currentLoc2High = LocationGlobalRelative(lat,lon,5)
                    vehicle.simple_goto(currentLoc2High)
                    time.sleep(6)
                    """
                    Burada normalde kalkip siseyi aldigini vs. kontrol etmesi ve ona gore alanlara ilerlemesi lazim
                    """
                    vehicle.simple_goto(yarimAlan)
                    time.sleep(8)
                    monkeyKing.turnOff()
                    cv2.destroyAllWindows()
                    break
    #####################################################################3
    if ( 180<=ax<=300  and   180<=ay<=300):
        merkez = True
    if (merkez == True):
        monkeyKing.turnOn()
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon  # O an ki konum
        currentLoc = LocationGlobalRelative(lat, lon, 1.2)
        vehicle.simple_goto(currentLoc)
        time.sleep(5)
        camera.close()
        currentLocHigh = LocationGlobalRelative(lat,lon,5)
        vehicle.simple_goto(currentLocHigh)
        time.sleep(6)
        """
        Burada normalde kalkip siseyi aldigini vs. kontrol etmesi ve ona gore alanlara ilerlemesi lazim
        
        th = Sensor()
        ilk , son = th.run()
        th1 = Output(ilk , son)
        th1.start()
        for duration in range (0,5):
            weight = float(th1.son)
            if weight > minWeight and weight < maxWeight:
                #Bu duruma gore git falan
        """        
        vehicle.simple_goto(doluAlan)
        time.sleep(8)
        monkeyKing.turnOff()
        break
    #####################################################################3
    



