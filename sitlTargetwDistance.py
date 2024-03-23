# -*- coding: utf-8 -*-
#!/usr/bin/env python

"""
May 10, 02:49AM

Amphetamine Levels : Critically Low

https://youtu.be/3HSpWQH4iRs
"""

from __future__ import print_function

import time,math

from dronekit import connect, VehicleMode, LocationGlobalRelative

import dronekit_sitl

sitl = dronekit_sitl.start_default() #dronekit-sitl'i default ayarlarda baslattik.

"""
Dronekit-sitl'i bu şekilde calistirdigimizda arastirmalarıma ve anladigima gore (yani kesin degil), mavproxy ile bağlantı dagitmamiz mümkün olmuyor ve yer istasyonunu dronekit-sitl'in 
bize sundugu diger tcp portlari olan 5761-2-3 portlarindan bagliyoruz.
"""

print("Connecting to vehicle on: %s" % ('tcp:127.0.0.1:5760',))

vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True) #sitl'e baglan

def arm():
    while vehicle.is_armable == 0:
        print ("Armlanmaya hazir degiliz.")
        time.sleep(1)
        
    print ("Drone armlaniyor...")
    """
    Aslında Auto olarak armlamamız ve pre-defined gorevi yaptırmamız gerek ancak suan elimizde gercek bir drone bile olmadigi icin GUIDED modda denemeler yapabiliriz.
    GUIDED mod aciklamasi : Navigates to single points commanded by GCS
    Diger ucus modlari hakkinda bilgi : http://ardupilot.org/copter/docs/flight-modes.html
    """
    vehicle.mode = VehicleMode("GUIDED")
    
    vehicle.armed = True
    
    while vehicle.armed == False:
        print("Armlaniyor...")
        time.sleep(1)
    
    print ("Drone arm : ",vehicle.armed)

arm() #Armlanma fonksiyonunu calistirdik.

"""
simple_takeoff'da vehicle gibi dronekit-python'dan geliyor. Kaynak kodları dronekit-python 
__init__.py dosyasında bulunabilir. Droneki'tin ana kaynak kodu bu dosyadir.
"""
def takeoff(alt):
    """
        Kaynak koddaki aciklama, asagdaki while dongusunu neden kullandigimi acikliyor.
        
        Take off and fly the vehicle to the specified altitude (in metres) and then wait for another command.

        .. note::

            This function should only be used on Copter vehicles.


        The vehicle must be in GUIDED mode and armed before this is called.

       !!!! Buraya dikkat. There is no mechanism for notification when the correct altitude is reached,
        and if another command arrives before that point (e.g. :py:func:`simple_goto`) it will be run instead.!!! 

        .. warning::

           Apps should code to ensure that the vehicle will reach a safe altitude before
           other commands are executed. A good example is provided in the guide topic :doc:`guide/taking_off`.

        :param alt: Target height, in metres.
    """
    vehicle.simple_takeoff(alt) #alt metreye droneu kaldir.
    
    while True:
        
        print("Yukseklik :", vehicle.location.global_relative_frame.alt)
        
        if vehicle.location.global_relative_frame.alt >= alt*0.95: #0,95 ile carptik cunku net veri alamayiz ve drone direkt yukseklikte duramaz.
           
           print("Hedef yukseklige ulasildi.")
           
           break #while true dongusunu yukseklige ulastigimiz icin bozuyoruz yoksa sonsuza dek devam eder
        
        time.sleep(1)

takeoff(10) #10metreye kalk
"""
Yukarida bahsettigim olay, GUIDED modda sadece tek tek waypoint (hedef nokta) verip, droneye oraya gitmesini soyleyebiliriz.
"""
wp1 = LocationGlobalRelative(-35.3621,149.167,10) #Rastgele bir yer verdim.
"""
simple_goto, drone'a su mavlik mesajini gonderir :  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

Diger mavlink mesajlari ve esktra bilgi  icin bkz : http://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html

 simple_goto'nun da dronekit komutu oldugunu soylemistim, kaynak kodda yazan aciklama 
her seyi anlatiyor :

    Go to a specified global location (:py:class:`LocationGlobal` or :py:class:`LocationGlobalRelative`).

        !!!!DIKKAT!!!There is no mechanism for notification when the target location is reached, and if another command arrives
        before that point that will be executed immediately.

        You can optionally set the desired airspeed or groundspeed (this is identical to setting
        :py:attr:`airspeed` or :py:attr:`groundspeed`). The vehicle will determine what speed to
        use if the values are not set or if they are both set.

        The method will change the :py:class:`VehicleMode` to ``GUIDED`` if necessary.

        .. code:: python

            # Set mode to guided - this is optional as the simple_goto method will change the mode if needed.
            vehicle.mode = VehicleMode("GUIDED")

            # Set the LocationGlobal to head towards
            a_location = LocationGlobal(-34.364114, 149.166022, 30)
            vehicle.simple_goto(a_location)

        :param location: The target location (:py:class:`LocationGlobal` or :py:class:`LocationGlobalRelative`).
        :param airspeed: Target airspeed in m/s (optional).
        :param groundspeed: Target groundspeed in m/s (optional).
"""

def uzaklikBul(location_1, location_2):
    """
    İki lokasyon arasi harita uzakligii bulur.
    Ardupilotun kodlarından buldum.
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat 		= location_2.lat - location_1.lat
    dlong		= location_2.lon - location_1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

vehicle.simple_goto(wp1)

while vehicle.mode.name=="GUIDED": #TRUE de diyebilirdik ancak kumandaya gecince dursun bari.
        kalanMesafe=uzaklikBul(vehicle.location.global_frame, wp1)
        print ("Hedefe Uzaklik : ", kalanMesafe)
        if kalanMesafe <= 1 : #Simple_goto'yu okursaniz bunun bizim gormemiz haric bir etkisi olmadigini anlarsiniz.
            print ("Hedefe ulasildi.")
            break;
        time.sleep(2)

