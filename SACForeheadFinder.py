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
        self.imageSize = imageSize
        self.minFaceSize = minFaceSize
        self.minEyesSize = minEyesSize
        # load frontal face  classifier
        self.faceDet = cv2.CascadeClassifier("/home/pi/SACLeptonRPi/haarcascade_frontalface_default.xml")
        self.eyesDet = cv2.CascadeClassifier("/home/pi/SACLeptonRPi/haarcascade_frontaleyes.xml")
        self.name = "Forehead"
        self.faceFound = False
        self.detectEyes = True

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
        start = int(round(time.time() * 1000))
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        print("Gray took: " + str(int(round(time.time() * 1000)) - start) + "ms")
        start = int(round(time.time() * 1000))
        rects = self.faceDet.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        print("Detect took: " + str(int(round(time.time() * 1000)) - start) + "ms")
      
        if len(rects) == 1 and rects[0][0] > 220 and (rects[0][0] + rects[0][2]) < 420:
            # only consider first face found.
            # rect comes in a tuple (x,y,w,h).
            # todo: check for biggest bounding box.
            # todo: check if face is in the middle!

            faceRect = rects[0]

            if showRois:
                self.showRect(image, faceRect, (200,255,150))
                self.faceFound = True

            if not self.detectEyes:
                self.tcRoi = (-1, -1, -1, -1)
                return False

            eyesRects = self.eyesDet.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
            if len(eyesRects) > 0:
                # Determine forehead                
                eyesRect = eyesRects[0]

                if showRois:
                    self.showRect(image, eyesRect, (255, 150, 200))

                x = eyesRect[0]
                y = faceRect[1]
                w = eyesRect[2]
                h = eyesRect[1] - faceRect[1]

                self.tcROI = (x, y, w, h)

                if showRois:
                    self.showRect(image, self.tcROI, (150, 255, 200))
                return True
            else:
                self.tcROI = (-1,-1,-1,-1)
                return False
        else:
            # no faces found
            self.tcROI = (-1,-1,-1,-1)
            self.faceFound = False
            return False

    def showRect(self, image, rect, color):
        x, y, w, h = rect
        cv2.rectangle(image,(x,y),(x + w,y + h), color, 2)

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

