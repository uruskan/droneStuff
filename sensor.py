import serial as sl
import time


class Sensor():

    def __init__(self, txt):
        self.txt = txt

    def sensorData(self):
        ser = sl.Serial('COM10', 9600)
        count = 0
        while count < 70:
            a = str(ser.readline())
            time.sleep(0.1)
            count = count + 1
            self.txt = a.split('-')
            #print a
            #print "ilk : " , self.txt[0]
            #print "son : " , self.txt[1]
            return self.txt