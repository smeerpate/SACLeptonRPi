#!/usr/bin/python3.7
# -*- coding: utf-8 -*-
import numpy as np
import cv2

########################################################
# Facefinder
########################################################
class FaceFider:

	def __init__(self, imageSize = (640,480), minFaceSize = (180,180)):
		self.imageSize = imageSize
		self.minFaceSize = minFaceSize
		
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
	def getTcFaceContours(self, image):
		gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		rects = faceDet.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=self.minFaceSize)
		if len(rects) > 0:
			# only consider first face found.
			# rect comes in a tuple (x,y,w,h).
			# todo: check for biggest bounding box.
			self.tcROI = rects[0]
	    	return TRUE
	    else:
	    	# no faces found
	    	self.tcROI = (-1,-1,-1,-1)
	    	return FALSE
	    		
	####################################################
	# Uses the transformation matrix to transform
	# the true color ROI to a thermal image ROI.
	# Returns the contour points of the thermal
	# image ROI.
	####################################################    		
	def getThFaceContours(self):  		
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
        P_slt = np.array([[self.tcROI.x],[self.tcROI.y],[1]]) # 'slt' source left top vector
        P_srt = np.array([[self.tcROI.x + self.tcROI.w],[self.tcROI.y],[1]]) # 'srt' source right top vector
        P_slb = np.array([[self.tcROI.x],[self.tcROI.y + self.tcROI.h],[1]]) # 'slb' source left bottom vector
        P_srb = np.array([[[self.tcROI.x + self.tcROI.w],[self.tcROI.y + self.tcROI.h],[1]]) # 'srb' source right bottom vector
        P_dlt = np.dot(self.M, P_slt)
        P_drt = np.dot(self.M, P_srt)
        P_dlb = np.dot(self.M, P_slb)
        P_drb = np.dot(self.M, P_srb)
        thROIPoints = np.array([P_dlt, P_drt, P_dlb, P_drb], dtype=np.float32)
        return(thROIPoints)