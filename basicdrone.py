# -*- coding: utf-8 -*-
from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
parser = argparse.ArgumentParser(description='Cok simple bir drone goto seysi')

hedef_adresi = "127.0.0.1:14550"
vehicle = connect(hedef_adresi, wait_ready=True)
def arm_and_takeoff(hedef_yukseklik):
    #Drone'u armla ve hedef yukseklige uc.
    
	# Otopilot bekle am
    while not vehicle.is_armable:
	
        time.sleep(1)
    
    #Gaydlı armlamak lazım simdi tabii
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # Tamam mıyız ?
    while not vehicle.armed:
    
        time.sleep(1)
    
    vehicle.simple_takeoff(hedef_yukseklik)  
    while True:
        
        if vehicle.location.global_relative_frame.alt >= hedef_yukseklik * 0.95:
            
			break
        time.sleep(1)

arm_and_takeoff(30)


vehicle.airspeed = 5


hedef1 = LocationGlobalRelative(39.867860, 32.731953, 40)
vehicle.simple_goto(hedef1, groundspeed=20)
vehicle.close()
