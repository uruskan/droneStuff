#!/usr/bin/python
import serial as sl
import time


ser = sl.Serial('COM10', baudrate=9600)
rawdata=[]
count = 0
a = str(ser.readline())
while count < 70:
    rawdata.append(str(ser.readline()))
    time.sleep(0.10)
    count = count + 1
   # if ("tamam" in str(ser.readline())):       Arduion'dan dogru veriler sayesinde 'tamam' geldigi zaman icin.
    #    print "MERHABA"
    txt = str(ser.readline()).split('-')  #Gelen verileri '-' gore ayirip yaziyor.
    print (str(ser.readline()))
    print   "ilk : " , txt[0]
    print   "son : " , txt[1]

#print rawdata


#/dev/tty.usbmodem1421