#!/usr/bin/env python
import open3d as o3d
import logging, coloredlogs
from string import digits

LOG_LEVEL = logging.DEBUG
logger = logging.getLogger('Input Output')
coloredlogs.install(level=LOG_LEVEL, logger=logger)

class InOut:
    
    def loadPointCloud(pointCloud):
        pc = o3d.io.read_point_cloud(pointCloud, format='auto', remove_nan_points=True, remove_infinite_points=True, print_progress=True)
        # print("Point cloud loaded with ", pc, " points")
        return pc
    
    def writeValues(QTpoints):
        f = open('Quality_Values.txt', 'w')
        QTpoints_str = str(QTpoints)
        f.write("Quality based on number of points is: ")
        f.write(QTpoints_str)
        f.close()