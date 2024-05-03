"""
Hand Module

"""

import cv2
import mediapipe as mp
import time
import numpy as np
from google.protobuf.json_format import MessageToDict


class handDetector():
    def __init__(self, mode=False, maxHands=2, modelComplexity=1, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.modelComplex = modelComplexity
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex, 
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils    
    # ham tra ve tay trai hoac phai hoac ca 2 khi dua len truoc cam 
    def chooseHand(self,img):
        handsType = []
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        choose = self.hands.process(imgRGB)
        if choose.multi_hand_landmarks != None:
            for hand in choose.multi_handedness:
                handType = hand.classification[0].label
                handsType.append(handType)
        return handsType       
    # hien thi khung xuong nhung chua xac dinh vi tri cac khop ngon
    def findHands(self, img, draw=True):
        #send rgb image to hands
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB) #process the frame
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw: #draw dots and connect them
                    self.mpDraw.draw_landmarks(img, handLms,
                                        self.mpHands.HAND_CONNECTIONS)
        return img
    # hien thi cac dau cham, the hien khop ngon tren khung xuong tay
        #Danh cho tay phai
    def findRightPosition(self, img, draw = True):
        lmList1 = []
        if self.results.multi_hand_landmarks:
            for idx,hand in enumerate(self.results.multi_hand_landmarks):
                lbl = self.results.multi_handedness[idx].classification[0].label
                if lbl == 'Left':
                    for id, lm in enumerate(hand.landmark):
                        # print(id, lm)
                        h, w, c = img.shape
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        # print(id, cx, cy)
                        lmList1.append([id, cx, cy])
                        if draw:
                            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
        return lmList1
        #Danh cho tay trai
    def findPosition(self, img, draw=True):
        lmList = []
        if self.results.multi_hand_landmarks:
            for idx,hand in enumerate(self.results.multi_hand_landmarks):
                lbl = self.results.multi_handedness[idx].classification[0].label
                if lbl == 'Right':
                    for id, lm in enumerate(hand.landmark):
                        # print(id, lm)
                        h, w, c = img.shape
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        # print(id, cx, cy)
                        lmList.append([id, cx, cy])
                        if draw:
                            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
        return lmList