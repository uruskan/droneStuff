
import numpy as np
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import math
import time
class director():
    def __init__(self, direction):
        self.direction = direction
  

    def ironmaiden(x, y, w, h, frame, t, color, a):
        cv2.putText(frame, color + str(t), (int(x), int(y + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255))
        M = cv2.moments(a)
        ax = int(M["m10"] / M["m00"])
        ay = int(M["m01"] / M["m00"])
        cv2.circle(frame, (ax, ay), 3, (255, 255, 255), -1)
        cv2.putText(frame, "center", (ax - 20, ay - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return (ax, ay)
        
    def moving(ax, ay):
        if ax > 240:
            self.direction = "right"
            print("saga git")
        elif ax < 240:
            self.direction = "left"
            print("sola git")
        if ay > 160:
            self.direction = "back"
            print("geri git")
        elif ay < 160:
            self.direction = "forward"
            print("ileri git")
    def measure():        
        camera = PiCamera()
        camera.resolution=(480,320)
        camera.framerate=32
        rawCapture = PiRGBArray(camera, size=(480,320))
        lower_red = np.array([130,150,150])
        upper_red = np.array([200,255,255])
        time.sleep(0.1)

        for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
            t=0
            shot=frame.array
            cv2.circle(shot,(240,160),3,(255,255,255),-1)
            blur=cv2.GaussianBlur(shot,(11,11),0)
            hsv=cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
            j= 0
            mask = cv2.inRange(hsv,lower_red,upper_red)
            kernelOPen = np.ones((20,20))
            mask=cv2.dilate(mask,kernelOPen,iterations=5)
            
            __, countours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if  j == 0:
                for a in countours:
                    peri = 0.01 * cv2.arcLength(a, True)
                    approx = cv2.approxPolyDP(a, peri, True)
                    if len(approx) >= 3:
                        if (cv2.contourArea(a) > 100):
                            x, y, w, h = cv2.boundingRect(a)
                            color = "Fire"
                            ax, ay = ironmaiden(x, y, w, h, shot, t, color, a)
                            moving(ax, ay)
                            
                    
            
            cv2.imshow('cam',shot)
            cv2.imshow('mask',mask)
            rawCapture.truncate(0)
            if cv2.waitKey(1)& 0xFF ==ord('q'):
                break
        
    



