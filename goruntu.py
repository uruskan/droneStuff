import sensor
import thread
from time import  sleep

txt12 = []

sen = sensor.Sensor(txt12)

def thread_metot(ad):
    i = 0
    while i < 100:
        print ad
        i = i + 1
        sleep(0.1)
    print("thread bitti")

try:
   thread.start_new_thread(thread_metot, (sen.sensorData() , ))  #Burdan emin değilim. Amacımız txt[0] ve txt[1] verilerilerini ayrı ayrı alabilmek
   thread.start_new_thread(sen.sensorData())
except:
   print "Error: unable to start thread"
while 1:
   pass
