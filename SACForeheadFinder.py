#!/usr/bin/python3.7
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
from RectangleOfInterestFinder import RectangleOfInterestFinder

########################################################
# Facefinder
########################################################
class ForeheadFinder(RectangleOfInterestFinder):

    def __init__(self, imageSize = (640,480), minFaceSize = (180,180), minEyesSize = (250,40)):
        self.minFaceSize = minFaceSize
        self.minEyesSize = minEyesSize
        # load frontal face  classifier
        #self.faceDet = cv2.CascadeClassifier("/home/pi/SACLeptonRPi/haarcascade_frontalface_default.xml")
        #self.eyesDet = cv2.CascadeClassifier("/home/pi/SACLeptonRPi/haarcascade_frontaleyes.xml")
        self.net = cv2.dnn.readNet('face-detection-adas-0001.xml', 'face-detection-adas-0001.bin')
        print("Read net completed")
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)
        print("Target set")
        self.name = "Forehead"
        self.faceFound = False

    ####################################################
    # Sets the the transformation matrix (M) for mapping
    # true color image onto thermal imaging image.
    # Is a numpy array e.g.:
    # np.array([[1.5689e-1, 8.6462e-3, -1.1660e+1],
    #           [1.0613e-4, 1.6609e-1, -1.4066e+1]])
    ####################################################
    def setTransformMatrix(self, transformMatrix):
        self.M = transformMatrix

    ####################################################
    # Takes a true color RGB image and looks for a face.
    # Return true if a face was found and false
    # if none was found.
    # Fills out the true color ROI.
    ####################################################
    def getTcContours(self, image, showRois):
        start = time.time()
        blob = cv2.dnn.blobFromImage(image, size=(640,480), ddepth=cv2.CV_8U)
        self.net.setInput(blob)
        out = self.net.forward()
        self.faceFound = False

        detections = []

        for detection in out.reshape(-1, 7):
            confidence = float(detection[2])
            if confidence > 0.5:
                xmin = int(detection[3] * image.shape[1])
                ymin = int(detection[4] * image.shape[0])
                xmax = int(detection[5] * image.shape[1])
                ymax = int(detection[6] * image.shape[0])

                faceWidth = xmax - xmin
                faceHeight = ymax - ymin                

                if xmin > 220 and xmax < 420 and faceWidth < 200 and faceWidth > 50 and faceHeight < 200 and faceHeight > 50:
                    detections.append((confidence, (xmin, ymin), (xmax, ymax)))

        # We only want 1 face
        if len(detections) == 1:
            detection = detections[0]            
            self.faceFound = True
            (xmin, ymin) = detection[1]
            (xmax, ymax) = detection[2]

            if showRois:                
                cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color=(255,0,0))
                cv2.putText(image, "Confidence: " + str(detection[0]), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)



            spacing = int((xmax - xmin) * 0.1)           

            x = xmin + spacing
            y = ymin
            w = xmax - xmin - spacing
            h = int((ymax - ymin) / 3)

            cv2.rectangle(image,(x,y),(x + w,y + h), (255, 255, 0), 1)

            self.tcROI = (x, y, w, h)

        timespan = (time.time() - start) * 1000
        print("Time to detect (all-in)(ms): " + str(timespan))
        self.tcROI = (-1,-1,-1,-1)
        return self.faceFound

    def getTcForeheadROIWidth(self):
        return self.tcROI[2]


    ####################################################
    # Uses the transformation matrix to transform
    # the true color ROI to a thermal image ROI.
    # Returns the contour points of the thermal
    # image ROI.
    ####################################################
    def getThContours(self):
        # transform the coordinates from true color image space to thermal image space using the affine transform matrix M
        # See https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
        # [[xt][yt]] = M . [[x][y][1]], with x,y=original coords and xt,yt=transformed coords.
        #
        # --->x
        # |
        # |   lt          rt
        # V   x------------x
        # y   |            |
        #     |            |
        #     |            |
        #     x------------x
        #     lb          rb
        #
        tcROI_x, tcROI_y, tcROI_w, tcROI_h = self.tcROI
        P_slt = np.array([[tcROI_x],[tcROI_y],[1]]) # 'slt' source left top vector
        P_srt = np.array([[tcROI_x + tcROI_w],[tcROI_y],[1]]) # 'srt' source right top vector
        P_slb = np.array([[tcROI_x],[tcROI_y + tcROI_h],[1]]) # 'slb' source left bottom vector
        P_srb = np.array([[tcROI_x + tcROI_w],[tcROI_y + tcROI_h],[1]]) # 'srb' source right bottom vector
        P_dlt = np.dot(self.M, P_slt)
        P_drt = np.dot(self.M, P_srt)
        P_dlb = np.dot(self.M, P_slb)
        P_drb = np.dot(self.M, P_srb)
        thROIPoints = np.array([P_dlt, P_drt, P_dlb, P_drb], dtype=np.float32)
        return(thROIPoints)

