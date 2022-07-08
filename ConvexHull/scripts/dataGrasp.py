#!/usr/bin/env python
import coloredlogs
import re, logging

LOG_LEVEL = logging.DEBUG
logger = logging.getLogger('Data')
coloredlogs.install(level=LOG_LEVEL, logger=logger)

class DataGrasp:


    def __init__(self, graspPointCloudPath):
        self.graspPointCloudPath = graspPointCloudPath


    def getGraspNumber(self,graspPointCloudPath):
        number = re.findall('\d+', graspPointCloudPath)
        number = int(number[0])
        # print(number)
        return number