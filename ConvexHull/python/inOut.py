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

    def changeGraspPointcloud(graspPointCloudPath, index):
        graspNumber_str = str(index)
        remove_digits = str.maketrans('', '', digits)
        noGraspNumberPath = graspPointCloudPath.translate(remove_digits)
        noFormatPath = noGraspNumberPath.replace('.pcd','')
        newgraspPointCloudPath = noFormatPath + graspNumber_str + '.pcd'
        return newgraspPointCloudPath
    
    def writeQualityValues(QTpoints, graspNumbers): #TODO 
        graspNumber = 1
        f = open('Quality_Values.txt', 'w')
        for qualityValue in QTpoints:
            QTpoints_str = str(float("{:.2f}".format(qualityValue)))
            grasp_str = str(graspNumber)
            f.write("Transfer Grasp Quality based on number of points for grasp " + grasp_str + " is: " + QTpoints_str + "\n")
            graspNumber += 1
        f.close()