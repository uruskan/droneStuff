 #!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import threading
import sensorDeneme
import logging
import time
import dronekit_sitl
import blackSnake
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal


#print('Connecting to vehicle on: %s' % connection_string)
#vehicle = connect(connection_string, wait_ready=True)
vehicle = connect('127.0.0.1:14550', wait_ready = True)

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


arm_and_takeoff(5)
time.sleep(1)

print("Set default/target airspeed to 10")
vehicle.airspeed = 10

#print("Going towards first point ...")
#point1 = LocationGlobalRelative(39.8720839, 32.7319155, 20)
#vehicle.simple_goto(point1)

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

def arama(p):
    print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
    vehicle.simple_goto(p, groundspeed=10)
    time.sleep(3)

p1 = [ LocationGlobalRelative(39.8720857, 32.7320695, 5), LocationGlobalRelative(39.8720841, 32.7321144, 5), LocationGlobalRelative(39.8720491, 32.7321386, 5)
,LocationGlobalRelative(39.8720723, 32.7321520, 5),LocationGlobalRelative(39.8720907, 32.7321407, 5),LocationGlobalRelative(39.8720538, 32.7321667, 5)
,LocationGlobalRelative(39.8720507,32.7321446, 5),LocationGlobalRelative(39.8720574,32.7321515, 5),LocationGlobalRelative(39.8720662,32.7321464, 5)
,LocationGlobalRelative(39.8720696, 32.7321412, 5),LocationGlobalRelative(39.8720619, 32.7321362, 5),LocationGlobalRelative(39.8720566, 32.7321381, 5)]

p2 = [ LocationGlobalRelative(39.8720857, 32.7320695, 5), LocationGlobalRelative(39.8720841, 32.7321862, 5), LocationGlobalRelative(39.8720677, 32.7322096, 5)
,LocationGlobalRelative(39.8720857, 32.7322358, 5),LocationGlobalRelative(39.8720996, 32.7322096, 5),LocationGlobalRelative(39.8720641,32.7322391, 5)
,LocationGlobalRelative(39.8720834, 32.7322324, 5),LocationGlobalRelative(39.8720777, 32.7322287, 5),LocationGlobalRelative(39.8720633, 32.7322304, 5)
,LocationGlobalRelative(39.8720661, 32.7322418, 5),LocationGlobalRelative(39.8720808, 32.7322442, 5),LocationGlobalRelative(39.8720821, 32.7322391, 5)]

class Output(threading.Thread):

    logging.basicConfig(level=logging.DEBUG, format='(%(threadName)-10s) %(message)s')

    def __init__(self, ilk , son):
        threading.Thread.__init__(self)
        self.ilk = ilk
        self.son = son
        return

    def run(self):
        logging.debug('Sonuc1: %s , sonuc2: %s ,  %s' , self.ilk , self.son, time.ctime(time.time()))
        return

alan_no = 0
gitti = True
bulundu = False
i1 = 0
direction = ""
while True:                     # İlk while
    t = sensorDeneme.Sensor()
    ilk, son = t.run()
    t2 = Output(ilk, son)
    t2.start()
    th_arama = threading.Thread(target= arama(p1[i1]))
    th_arama.start()
    ourDirector = blackSnake.Director()
    directorThread = threading.Thread(target=ourDirector.measure,args=())
    directorThread.start()
    print (ourDirector.direction) 
    i1 = i1 + 1
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon   #O an ki konum
    point = LocationGlobalRelative(lat, lon, 1)
    if (gitti == True):
        if (t2.ilk >= '100' and t2.ilk <= '150'):     #Gercekte olmasini bekledigimiz rakamlar 't2.ilk >= 465 & t2.ilk <= 480'!!! (5 metrede ucarsak)
            logging.debug('SİSE BULUNDU!!!')
            logging.debug('TAMAM %s', vehicle.location.global_relative_frame.alt)
            print("Going toward second point...")
            print(lat)
            print(lon)
            bulundu = True
            t2.join()
            print(bulundu)

        if (bulundu == True):
            vehicle.simple_goto(point, 10)
            time.sleep(5)  #Elektro magnete gore burası degisebilir.

            # Elektro magnet calistirma fonksiyonları !!!

            point_son = LocationGlobalRelative(lat, lon, 5)
            vehicle.simple_goto(point_son, groundspeed=10)   #Siseyi alip yukari 5 metreye cikiyor.
            time.sleep(10)

            print("BASLADI")
            time.sleep(1)

            raw_data = []
            i2 = 0
            while i2 < 10:
                t = sensorDeneme.Sensor()
                ilk2, son2 = t.run()
                t3 = Output(ilk2, son2)
                t3.start()
                a = float((t3.son).replace("\n", ""))
                raw_data.append(a)
                time.sleep(1)
                i2 = i2 + 1
            print(raw_data)
            toplam = raw_data[0] + raw_data[1] + raw_data[2] + raw_data[3] + raw_data[4] + raw_data[5] + raw_data[6] + raw_data[7] + raw_data[8] + raw_data[9]
            ortalama = toplam / 10
            print(ortalama)

            if ( ortalama > 50.0):
                alan_dolu(3, 1)
                # Elektro magnet birakma fonksiyonları!!!
                alan_no = 1

            else:
                alan_yarim(3, 1)
                # Elektro magnet ile alana siseyi birakti
                alan_no = 2

            bulundu =False
            gitti = False

            print("Birinci sise bitti!!!")
            break

        else:    #(t2.ilk < '200' and t2.ilk > '250')
            pass
    else:
        pass

print("Ikinci while basladi!!!")

i3 = 0
while True:                                    #Ikinci while
    t = sensorDeneme.Sensor()
    ilk, son = t.run()
    t2 = Output(ilk, son)
    t2.start()
    th_arama = threading.Thread(target= arama(p2[i3]))
    th_arama.start()
    i3 = i3 + 1
    print(gitti)
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon   #O an ki konum
    point = LocationGlobalRelative(lat, lon, 1)
    if (t2.ilk >= '100' and t2.ilk <= '150'):     #Gercekte olmasini bekledigimiz rakamlar 't2.ilk >= 465 & t2.ilk <= 480'!!! (5 metrede ucarsak)
        logging.debug('SİSE BULUNDU!!!')
        logging.debug('TAMAM %s', vehicle.location.global_relative_frame.alt)
        print("Going toward second point...")
        bulundu = True
        t2.join()

    if (bulundu == True):
        vehicle.simple_goto(point, 10)
        time.sleep(10)  #Elektro magnete gore burası degisebilir.

            # Elektro magnet calistirma fonksiyonları !!!

        point_son = LocationGlobalRelative(lat, lon, 5)
        vehicle.simple_goto(point_son, groundspeed=10)   #Siseyi alip yukari 5 metreye cikiyor.
        time.sleep(10)

        if ( alan_no == 1):
            alan_yarim(3, 1)
                # Elektro magnet birakma fonksiyonları!!!
            pass

        else:
            alan_dolu(3, 1)
            # Elektro magnet ile alana siseyi birakti
            pass

        bulundu =False

        print("BITTI!!!")
        break

    else:
        pass

print("RTL...")
vehicle.mode = VehicleMode("RTL")

#Goruntu isleme fonksiyonlarinin entegresi
#Filtreleme
#elekrtro magnet fonksiyonları
