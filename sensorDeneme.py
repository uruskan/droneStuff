import threading
import thread
import logging
import time
import serial as sl

class Sensor(threading.Thread):
    logging.basicConfig(level=logging.DEBUG,format='(%(threadName)-10s) %(message)s', )

    def __init__(self):
        threading.Thread.__init__(self)
        return

    def run(self):
        ser = sl.Serial('COM10' , baudrate=9600)
        a = str(ser.readline())
        b = a.split('-')
        logging.debug('Veriler: %s , %s' ,a , time.ctime(time.time()) )
        #logging.debug('Son Hal: %s', b)  #Duzelt
        time.sleep(0.1)
        return b[0], b[1]

