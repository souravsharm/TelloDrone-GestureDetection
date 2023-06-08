import cv2
from cvzone.HandTrackingModule import HandDetector
from cvzone.ClassificationModule import Classifier
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class Gesture(Node):
    def __init__(self):
        super().__init__('gesturenode')
        self.pub = self.create_publisher(String, 'gestures', 1)
        self.create_timer(1, self.cb)
        self.command = ""
        self.detector = HandDetector(maxHands=1)
        model_path = os.path.join(os.path.dirname(__file__), 'Model/keras_model.h5')
        labels_path = os.path.join(os.path.dirname(__file__), 'Model/labels.txt')
        self.classifier = Classifier(model_path, labels_path)
        self.offset = 20
        self.imgSize = 300

    def cb(self):
        command = self.findGesture()
        if command != "":
            msg = String()
            msg.data = command
            self.pub.publish(msg)
    

    def findGesture(self):
        cap = cv2.VideoCapture(0)
        labels = ["takeoff", "land", "right"]
        prevIndex = -1
        counter =0

        while True:
            success, img = cap.read()
            imgOutput = img.copy()
            hands, img = self.detector.findHands(img)
            if hands:
                hand = hands[0]
                x, y, w, h = hand['bbox']

                imgWhite = np.ones((self.imgSize, self.imgSize, 3), np.uint8) * 255
                imgCrop = img[y - self.offset:y + h + self.offset, x - self.offset:x + w + self.offset]

                imgCropShape = imgCrop.shape

                aspectRatio = h / w

                if aspectRatio > 1:
                    k = self.imgSize / h
                    wCal = math.ceil(k * w)
                    imgResize = cv2.resize(imgCrop, (wCal, self.imgSize))
                    imgResizeShape = imgResize.shape
                    wGap = math.ceil((self.imgSize - wCal) / 2)
                    imgWhite[:, wGap:wCal + wGap] = imgResize
                    prediction, index = self.classifier.getPrediction(imgWhite, draw=False)
                    
                    if(prevIndex==index):
                        counter=counter+1
                    else:
                        counter=0

                    prevIndex = index
                    if(counter>10):
                        print("from if:  Command: ", labels[index], index)
                        counter=0
                        return str(labels[index])

                else:
                    k = self.imgSize / w
                    hCal = math.ceil(k * h)
                    imgResize = cv2.resize(imgCrop, (self.imgSize, hCal))
                    imgResizeShape = imgResize.shape
                    hGap = math.ceil((self.imgSize - hCal) / 2)
                    imgWhite[hGap:hCal + hGap, :] = imgResize
                    prediction, index = self.classifier.getPrediction(imgWhite, draw=False)
                    if(prevIndex==index):
                        counter=counter+1
                    else:
                        counter=0
                        
                    prevIndex = index
                    if(counter>5):
                        print("from else:   Command",  labels[index], index)
                        counter=0
                        return str(labels[index])

                cv2.rectangle(imgOutput, (x - self.offset, y - self.offset-50),
                            (x - self.offset+90, y - self.offset-50+50), (255, 0, 255), cv2.FILLED)
                cv2.putText(imgOutput, labels[index], (x, y - 26), cv2.FONT_HERSHEY_COMPLEX, 1.7, (255, 255, 255), 2)
                cv2.rectangle(imgOutput, (x-self.offset, y-self.offset),
                            (x + w+self.offset, y + h+self.offset), (255, 0, 255), 4)

            cv2.imshow("Image", imgOutput)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    gesture_node = Gesture()
    rclpy.spin(gesture_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
