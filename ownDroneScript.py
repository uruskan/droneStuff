from __future__ import print_function
from pymavlink import mavutil, mavwp
from pymavlink.dialects.v10 import ardupilotmega

import time
from dronekit import connect, VehicleMode
import math
arac_adresi = "udp:127.0.0.1:1450"

print('Hedef Arac : %s' % arac_adresi)

vehicle = connect(arac_adresi, wait_ready=True)

while not vehicle.is_armable:
        
    print(" Waiting for vehicle to initialise...")
        
    time.sleep(1)
    
print("Motorlar silahlandiriliyor.") #just joking
    
vehicle.mode = VehicleMode("GUIDED")
    
vehicle.armed = True
    
while not vehicle.armed:
	    
    print("Armladik aslinda ama armlanmamis mi ki?, bekleniyor...") #Sanirim vehicle.armed is not just a variable so we need to check for  if we really armed it.
        
    time.sleep(1)
    
print("Armlandi.")



print(vehicle.mode)	

def takeoff(self, alt):
        """
        Take off and fly the vehicle to the specified altitude (in metres) and then wait for another command.

        .. note::

            This function should only be used on Copter vehicles.


        The vehicle must be in GUIDED mode and armed before this is called.

        There is no mechanism for notification when the correct altitude is reached,
        and if another command arrives before that point (e.g. :py:func:`simple_goto`) it will be run instead.

        .. warning::

           Apps should code to ensure that the vehicle will reach a safe altitude before
           other commands are executed. A good example is provided in the guide topic :doc:`guide/taking_off`.

        :param alt: Target height, in metres.
        """
        if alt is not None:
            altitude = float(alt)
            if math.isnan(altitude) or math.isinf(altitude):
                raise ValueError("Altitude was NaN or Infinity. Please provide a real number")
            self._master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, altitude)
			
											   

takeoff(vehicle,50)

time.sleep(10)

print("Vehicle close")

vehicle.close()