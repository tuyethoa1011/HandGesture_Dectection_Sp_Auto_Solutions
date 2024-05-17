import sys
# Thư viện để nhận diện cử chỉ tay
import cv2
import time
import os
from google.protobuf.json_format import MessageToDict
import numpy as np
import hand as htm
import mediapipe as mp
from picamera2 import Picamera2
# Importing dependency Libraries to handle serial COM
import serial 
import serial.tools.list_ports


#Importing depency Libraries for thread
from threading import Thread

from gpiozero import LED

#uart permission: os.system("sudo chmod a+rw /dev/serial0")

redNoticeLed = LED(19) #GPIO Led canh bao viec su dung 2 tay de nhan dien
firstBitLed = LED(26) 
secondBitLed = LED(27) 
thirdBitLed = LED(25) 
fourthBitLed = LED(24)
leftHandLed = LED(13)
rightHandLed = LED(22)

bootLed = LED(16)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
#set request fps
picam2.preview_configuration.controls.FrameRate==20
picam2.start()
#determine fps - how long it takes to do 1 frame - loop time - time to get (process) 1 frame
fps=0
pos= (30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
myColor=(0,0,255)
weight=3

#khai báo sử dụng thư viện và mặc định define
#cap = cv2.VideoCapture("/dev/video0")
detector = htm.handDetector(detectionCon=0.55) 
fingerid = [4,8,12,16,20]
lefthand_value = ''
righthand_value = ''


ser = serial.Serial(
	port = '/dev/serial0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1
)

#ser = serial.Serial ("/dev/serial0", 115200)    #Open port with baud rate

#Bien co
ledFlag = 0
bootFlag = 0

############################################   Start Left Hand   ####################################################################
# Hàm nhận diện cử chỉ tay trái
def taytrai():
    if len(lmList)!=0:  
        songontay = None
            # Neu la tay trai
        if lmList[fingerid[0]][1] < lmList[fingerid[4]][1]:
            #Lenh 1 (Nam tay, khong thay ngon nao)
            if (lmList[fingerid[0]][1] > lmList[fingerid[0] - 1][1] and
                lmList[fingerid[1]][2] > lmList[fingerid[1] - 2][2] and
                lmList[fingerid[2]][2] > lmList[fingerid[2] - 2][2] and
                lmList[fingerid[3]][2] > lmList[fingerid[3] - 2][2] and
                lmList[fingerid[4]][2] > lmList[fingerid[4] - 2][2]):
                songontay = 0
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            #Lenh 2(Chi mot ngon tro)
            elif (lmList[fingerid[0]][1] > lmList[fingerid[0] - 1][1] and
                lmList[fingerid[1]][2] < lmList[fingerid[1] - 2][2] and
                lmList[fingerid[2]][2] > lmList[fingerid[2] - 2][2] and
                lmList[fingerid[3]][2] > lmList[fingerid[3] - 2][2] and
                lmList[fingerid[4]][2] > lmList[fingerid[4] - 2][2]):
                songontay = 1
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 3 (1 ngon tro, 1 ngon giua)
            elif (lmList[fingerid[1]][2] < lmList[fingerid[1]-2][2] and
                lmList[fingerid[4]][2] > lmList[fingerid[4]-2][2] and
                lmList[fingerid[2]][2] < lmList[fingerid[2]-2][2] and
                lmList[fingerid[3]][2] > lmList[fingerid[3]-2][2] and
                lmList[fingerid[0]][1] > lmList[fingerid[0] - 1][1]):
                songontay = 2
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 4 (1 ngon tro, 1 ngon giua, 1 ngon ap ut )
            elif (lmList[fingerid[1]][2] < lmList[fingerid[1]-2][2] and
                lmList[fingerid[2]][2] < lmList[fingerid[2]-2][2] and
                lmList[fingerid[3]][2] < lmList[fingerid[3]-2][2] and
                lmList[fingerid[4]][2] > lmList[fingerid[4]-2][2] and
                lmList[fingerid[0]][1] > lmList[fingerid[0] - 1][1]):
                songontay = 3
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 5 (4 ngon dai )
            elif (lmList[fingerid[1]][2] < lmList[fingerid[1]-2][2] and
                lmList[fingerid[2]][2] < lmList[fingerid[2]-2][2] and
                lmList[fingerid[4]][2] < lmList[fingerid[4]-2][2] and
                lmList[fingerid[3]][2] < lmList[fingerid[3]-2][2] and
                lmList[fingerid[0]][1] > lmList[fingerid[0] - 1][1]):
                songontay = 4
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 6 (Chi ngon cai)
            elif (lmList[fingerid[0]][1] < lmList[fingerid[0] - 1][1] and
                lmList[fingerid[1]][2] > lmList[fingerid[1] - 2][2] and
                lmList[fingerid[2]][2] > lmList[fingerid[2] - 2][2] and
                lmList[fingerid[3]][2] > lmList[fingerid[3] - 2][2] and
                lmList[fingerid[4]][2] > lmList[fingerid[4] - 2][2]):
                songontay = 5
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 7 (xoe ban tay)
            elif (lmList[fingerid[1]][2] < lmList[fingerid[1]-2][2] and
                lmList[fingerid[2]][2] < lmList[fingerid[2]-2][2] and
                lmList[fingerid[3]][2] < lmList[fingerid[3]-2][2] and
                lmList[fingerid[4]][2] < lmList[fingerid[4]-2][2] and
                lmList[fingerid[0]][1] < lmList[fingerid[0] - 1][1]):
                songontay = 6             
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 8 (Chi ngon cai va ngon tro)
            elif (lmList[fingerid[0]][1] < lmList[fingerid[0] - 1][1] and
                lmList[fingerid[1]][2] < lmList[fingerid[1] - 2][2] and
                lmList[fingerid[2]][2] > lmList[fingerid[2] - 2][2] and
                lmList[fingerid[3]][2] > lmList[fingerid[3] - 2][2] and
                lmList[fingerid[4]][2] > lmList[fingerid[4] - 2][2]):
                songontay = 7
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 9 (Chi ngon cai, ngon tro va ngon giua )
            elif (lmList[fingerid[0]][1] < lmList[fingerid[0] - 1][1] and
                lmList[fingerid[1]][2] < lmList[fingerid[1] - 2][2] and
                lmList[fingerid[2]][2] < lmList[fingerid[2] - 2][2] and
                lmList[fingerid[3]][2] > lmList[fingerid[3] - 2][2] and
                lmList[fingerid[4]][2] > lmList[fingerid[4] - 2][2]):
                songontay = 8
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            else:
                songontay = None
        else: 
          songontay = None
        return songontay
                    ############################################   End Left Hand   ##############################################################



                    ############################################   Start Right Hand   ###########################################################
# Hàm nhận diện cử chỉ tay phải
def tayphai():
     # Neu la tay phai
    if len(lmList1)!=0:
        songontay = None
        if lmList1[fingerid[0]][1] > lmList1[fingerid[4]][1]:
            # Lenh 1 (Nam tay, khong thay ngon nao)
            if (lmList1[fingerid[0]][1] < lmList1[fingerid[0] - 1][1] and
                lmList1[fingerid[1]][2] > lmList1[fingerid[1] - 2][2] and
                lmList1[fingerid[2]][2] > lmList1[fingerid[2] - 2][2] and
                lmList1[fingerid[3]][2] > lmList1[fingerid[3] - 2][2] and
                lmList1[fingerid[4]][2] > lmList1[fingerid[4] - 2][2]) :
                songontay = 0
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 2 (Duy nhat 1 ngon tro)
            elif (lmList1[fingerid[0]][1] < lmList1[fingerid[0] - 1][1] and
                lmList1[fingerid[1]][2] < lmList1[fingerid[1] - 2][2] and
                lmList1[fingerid[2]][2] > lmList1[fingerid[2] - 2][2] and
                lmList1[fingerid[3]][2] > lmList1[fingerid[3] - 2][2] and
                lmList1[fingerid[4]][2] > lmList1[fingerid[4] - 2][2]) :
                songontay = 1
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 3 (1 ngon tro, 1 ngon giua)
            elif (lmList1[fingerid[1]][2] < lmList1[fingerid[1]-2][2] and
                lmList1[fingerid[4]][2] > lmList1[fingerid[4]-2][2] and
                lmList1[fingerid[2]][2] < lmList1[fingerid[2]-2][2] and
                lmList1[fingerid[3]][2] > lmList1[fingerid[3]-2][2] and
                lmList1[fingerid[0]][1] < lmList1[fingerid[0] - 1][1]):
                songontay = 2
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 4 (1 ngon tro, 1 ngon giua, 1 ngon ap ut)
            elif (lmList1[fingerid[1]][2] < lmList1[fingerid[1]-2][2] and
                lmList1[fingerid[2]][2] < lmList1[fingerid[2]-2][2] and
                lmList1[fingerid[3]][2] < lmList1[fingerid[3]-2][2] and
                lmList1[fingerid[4]][2] > lmList1[fingerid[4]-2][2] and
                lmList1[fingerid[0]][1] < lmList1[fingerid[0] - 1][1]):
                songontay = 3
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 5 (4 ngon dai)
            elif (lmList1[fingerid[1]][2] < lmList1[fingerid[1]-2][2] and
                lmList1[fingerid[2]][2] < lmList1[fingerid[2]-2][2] and
                lmList1[fingerid[4]][2] < lmList1[fingerid[4]-2][2] and
                lmList1[fingerid[3]][2] < lmList1[fingerid[3]-2][2] and
                lmList1[fingerid[0]][1] < lmList1[fingerid[0] - 1][1]) :
                songontay = 4
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 6 ( Chi ngon cai)
            elif (lmList1[fingerid[0]][1] > lmList1[fingerid[0] - 1][1] and
                lmList1[fingerid[1]][2] > lmList1[fingerid[1] - 2][2] and
                lmList1[fingerid[2]][2] > lmList1[fingerid[2] - 2][2] and
                lmList1[fingerid[3]][2] > lmList1[fingerid[3] - 2][2] and
                lmList1[fingerid[4]][2] > lmList1[fingerid[4] - 2][2]) :
                songontay = 5
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 7 (Ca 5 ngon)
            elif (lmList1[fingerid[1]][2] < lmList1[fingerid[1]-2][2] and
                lmList1[fingerid[2]][2] < lmList1[fingerid[2]-2][2] and
                lmList1[fingerid[3]][2] < lmList1[fingerid[3]-2][2] and
                lmList1[fingerid[4]][2] < lmList1[fingerid[4]-2][2] and
                lmList1[fingerid[0]][1] > lmList1[fingerid[0] - 1][1]):
                songontay = 6
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 8 (Chi ngon cai va ngon tro)
            elif (lmList1[fingerid[0]][1] > lmList1[fingerid[0] - 1][1] and
                lmList1[fingerid[1]][2] < lmList1[fingerid[1] - 2][2] and
                lmList1[fingerid[2]][2] > lmList1[fingerid[2] - 2][2] and
                lmList1[fingerid[3]][2] > lmList1[fingerid[3] - 2][2] and
                lmList1[fingerid[4]][2] > lmList1[fingerid[4] - 2][2]) :
                songontay = 7
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            # Lenh 9 (Chi ngon cai, ngon tro va ngon giua)
            elif (lmList1[fingerid[0]][1] > lmList1[fingerid[0] - 1][1] and
                lmList1[fingerid[1]][2] < lmList1[fingerid[1] - 2][2] and
                lmList1[fingerid[2]][2] < lmList1[fingerid[2] - 2][2] and
                lmList1[fingerid[3]][2] > lmList1[fingerid[3] - 2][2] and
                lmList1[fingerid[4]][2] > lmList1[fingerid[4] - 2][2]) :
                songontay = 8
                #cv2.rectangle(frame1,(0,200),(100,300),(0,255,0),-1)
                #cv2.putText(frame1,str(songontay),(30,280),cv2.FONT_HERSHEY_PLAIN,5,(255,0,0),5)
            else:
              songontay = None
        else:
          songontay = None
        return songontay
###############################################   End Right Hand  ###############################################################

def transmitLHData(): #LH: LeftHand
    while True:
      if ledFlag == 0:
        redNoticeLed.off()
        if lefthand_value == 0:
          leftHandLed.on()
          rightHandLed.off()
          
          firstBitLed.off()
          secondBitLed.off()
          thirdBitLed.off()
          fourthBitLed.off()
          #nen co mot den de phan biet dang nhan dien tay trai hay tay phai
          ser.write(str.encode('0'))
        elif lefthand_value == 1:
          leftHandLed.on()
          rightHandLed.off()
          firstBitLed.on()
          secondBitLed.off()
          thirdBitLed.off()
          fourthBitLed.off()
          
          ser.write(str.encode('1'))
        elif lefthand_value == 2:
          leftHandLed.on()
          rightHandLed.off()
          
          firstBitLed.off()
          secondBitLed.on()
          thirdBitLed.off()
          fourthBitLed.off()
          
          ser.write(str.encode('2'))
        elif lefthand_value == 3:
          leftHandLed.on()
          rightHandLed.off()
          
          firstBitLed.on()
          secondBitLed.on()
          thirdBitLed.off()
          fourthBitLed.off()
          
          ser.write(str.encode('3'))
        elif lefthand_value == 4:
          leftHandLed.on()
          rightHandLed.off()
          
          firstBitLed.off()
          secondBitLed.off()
          thirdBitLed.on()
          fourthBitLed.off()
          
          ser.write(str.encode('4'))
        elif lefthand_value == 5:
          leftHandLed.on()
          rightHandLed.off()
          
          firstBitLed.on()
          secondBitLed.off()
          thirdBitLed.on()
          fourthBitLed.off()
          
          ser.write(str.encode('6'))
        elif lefthand_value == 6:
          leftHandLed.on()
          rightHandLed.off()
          
          firstBitLed.off()
          secondBitLed.on()
          thirdBitLed.on()
          fourthBitLed.off()
          
          ser.write(str.encode('5'))
        elif lefthand_value == 7:
          leftHandLed.on()
          rightHandLed.off()
          
          firstBitLed.on()
          secondBitLed.on()
          thirdBitLed.on()
          fourthBitLed.off()
          
          ser.write(str.encode('7'))
        elif lefthand_value == 8:
          leftHandLed.on()
          rightHandLed.off()
          
          firstBitLed.off()
          secondBitLed.off()
          thirdBitLed.off()
          fourthBitLed.on()
          
          ser.write(str.encode('8'))
      elif ledFlag == 1:
        #bat den bao nhan dien 2 tay
        redNoticeLed.on()
        # tat den bao hieu
        leftHandLed.off()
        rightHandLed.off()
          
        firstBitLed.off()
        secondBitLed.off()
        thirdBitLed.off()
        fourthBitLed.off()
      time.sleep(1)


def transmitRHData(): #RH: RightHand
    while True:
      if ledFlag == 0:
        redNoticeLed.off()
        if righthand_value == 0:
          leftHandLed.off()
          rightHandLed.on()
          
          firstBitLed.off()
          secondBitLed.off()
          thirdBitLed.off()
          fourthBitLed.off()
          
          ser.write(str.encode('0'))
        elif righthand_value == 1:
          leftHandLed.off()
          rightHandLed.on()
          
          firstBitLed.on()
          secondBitLed.off()
          thirdBitLed.off()
          fourthBitLed.off()
          
          ser.write(str.encode('1'))
        elif righthand_value == 2:
          leftHandLed.off()
          rightHandLed.on()
          
          firstBitLed.off()
          secondBitLed.on()
          thirdBitLed.off()
          fourthBitLed.off()
          
          ser.write(str.encode('2'))
        elif righthand_value == 3:
          leftHandLed.off()
          rightHandLed.on()
          
          firstBitLed.on()
          secondBitLed.on()
          thirdBitLed.off()
          fourthBitLed.off()
          
          ser.write(str.encode('3'))
        elif righthand_value == 4:
          leftHandLed.off()
          rightHandLed.on()
          
          firstBitLed.off()
          secondBitLed.off()
          thirdBitLed.on()
          fourthBitLed.off()
          
          ser.write(str.encode('4'))
        elif righthand_value == 5:
          leftHandLed.off()
          rightHandLed.on()
          
          firstBitLed.on()
          secondBitLed.off()
          thirdBitLed.on()
          fourthBitLed.off()
          
          ser.write(str.encode('6'))
        elif righthand_value == 6:
          leftHandLed.off()
          rightHandLed.on()
          
          firstBitLed.off()
          secondBitLed.on()
          thirdBitLed.on()
          fourthBitLed.off()
          
          ser.write(str.encode('5'))
        elif righthand_value == 7:
          leftHandLed.off()
          rightHandLed.on()
          
          firstBitLed.on()
          secondBitLed.on()
          thirdBitLed.on()
          fourthBitLed.off()
          
          ser.write(str.encode('7')) 
        elif righthand_value == 8:
          leftHandLed.off()
          rightHandLed.on()
          
          firstBitLed.off()
          secondBitLed.off()
          thirdBitLed.off()
          fourthBitLed.on()
          
          ser.write(str.encode('8'))
      elif ledFlag == 1:
        #bat den bao nhan dien 2 tay
        redNoticeLed.on()
        #tat het den bao hieu 
        leftHandLed.off()
        rightHandLed.off()
          
        firstBitLed.off()
        secondBitLed.off()
        thirdBitLed.off()
        fourthBitLed.off()
      time.sleep(1)

thread1 = Thread(target = transmitLHData) 
thread2 = Thread(target = transmitRHData) 

thread1.start()
thread2.start()

if bootFlag == 0:
  bootLed.on()
  bootFlag = 1

while True:
    tStart=time.time()
    frame = picam2.capture_array()
    #ret,frame = cap.read()
    handsType = detector.chooseHand(frame)
    frame1 = detector.findHands(frame)
    lmList =detector.findPosition(frame, draw= False)
    lmList1 = detector.findRightPosition(frame, draw = False)
    #print("tay trai" ,lmList) # tay trai
    #print("tayphai",lmList1) # tay phai
    #cv2.putText(frame,str(int(fps)) + ' FPS',pos,font,height,myColor,weight)
    cv2.imshow("Camera", frame)
    if (handsType == ['Left', 'Right'] or handsType == ['Right', 'Left']):
        print("2 hand warning\n")
        ledFlag = 1
    else:
        ledFlag = 0
        lefthand_value = taytrai()
        righthand_value  = tayphai()
        #print(lefthand_value, ' ', righthand_value)
    if cv2.waitKey(1) == ord("q"):
        break
    tEnd=time.time()
    loopTime = tEnd-tStart
    #fps=1/loopTime
    #add low pass filter for terminating noise
    fps=.9*fps+.1*(1/loopTime)
    #print(int(fps))
    #time.sleep(0.1)
cap.release()
cv2.destroyAllWindows()

