#!/usr/bin/env python
from open3d import *
import logging, coloredlogs
from string import digits

LOG_LEVEL = logging.DEBUG
logger = logging.getLogger('Input Output')
coloredlogs.install(level=LOG_LEVEL, logger=logger)

class InOut:


    def __init__(self, graspPointCloudPath, objectPointCloudPath, transformationFile):
        self.graspPointCloudPath = graspPointCloudPath
        self.objectPointCloudPath = objectPointCloudPath
        self.transformationFile = transformationFile


    def loadPointCloud(self, pointCloud):
        pc = io.read_point_cloud(pointCloud, format='auto', remove_nan_points=True, remove_infinite_points=True, print_progress=True)
        return pc

    def changeGraspPointcloud(self, graspPointCloudPath, graspNumber):
        # remove_digits = str.maketrans('', '', digits)
        # noGraspNumberPath = graspPointCloudPath.translate(remove_digits)
        
        print(noGraspNumberPath)