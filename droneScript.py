# -*- coding: utf-8 -*-
from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
parser = argparse.ArgumentParser(description='Cok simple bir drone goto seysi')
parser.add_argument('--hedef',

                    help="Hedef aracin adresini gir am")
args = parser.parse_args()
hedef_adresi = args.hedef
print('Drone\'a baglaniliyor: %s' % hedef_adresi)
vehicle = connect(hedef_adresi, wait_ready=True)
def arm_and_takeoff(hedef_yukseklik):
    #Drone'u armla ve hedef yukseklige uc.
    print("Arm oncesi kontroller.")
	# Otopilot bekle am
    while not vehicle.is_armable:
	print("Drone mu tost makinesi mi belli degil aq, drone armable degil")
        time.sleep(1)
    print("Motorlar silahlandırılıyor.")
    #Gaydlı armlamak lazım simdi tabii
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # Tamam mıyız ?
    while not vehicle.armed:
        print("Harbiden tost makinesi hala hazir degil, bekleniyor..")
        time.sleep(1)
    print("Ignition.. , Kalkis")
    vehicle.simple_takeoff(hedef_yukseklik)  
		while True:
        print(" Yukseklik: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= hedef_yukseklik * 0.95:
            print("Hedef yukseklige ulasildi.")
        break
        time.sleep(1)

arm_and_takeoff(30)

print("Airspeed 5 olarak ayarlandi.")
vehicle.airspeed = 5

print("Hedef1'e gidiliyor...")
hedef1 = LocationGlobalRelative(39.867860, 32.731953, 40)
vehicle.simple_goto(hedef1, groundspeed=20)
time.sleep(30)
print("Baslangica geri donuluyor")
vehicle.mode = VehicleMode("RTL")

print("Drone'u kapat.")
vehicle.close()
