#!/usr/bin/env python
import coloredlogs
import re, logging
import numpy as np
import array as arr

LOG_LEVEL = logging.DEBUG
logger = logging.getLogger('Data')
coloredlogs.install(level=LOG_LEVEL, logger=logger)

class DataGrasp:

    def __init__(self, graspPointCloudPath):
        self.graspPointCloudPath = graspPointCloudPath

    def getGraspNumber(self,graspPointCloudPath):
        number = re.findall('\d+', graspPointCloudPath)
        number = int(number[0])
        return number
    
    def getMaxQuality(self,graspQualityValues):
        maxQuality = max(graspQualityValues)
        pose = graspQualityValues.index(maxQuality)
        bestQualityGraspNumber = pose + 1
        graspNumber_str = str(bestQualityGraspNumber)
        maxQuality_str = str(float("{:.2f}".format(maxQuality)))
        print("Grasp " + graspNumber_str + " has the max quality with " + maxQuality_str)
        return maxQuality, bestQualityGraspNumber