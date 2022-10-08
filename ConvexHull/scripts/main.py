#!/usr/bin/env python
import argparse, time
import logging, coloredlogs
from mvbb import Quality

LOG_LEVEL = logging.DEBUG
logger = logging.getLogger('Quality Measures')
coloredlogs.install(level=LOG_LEVEL, logger=logger)

parser = argparse.ArgumentParser(description='Quality measures for transferring objects')
parser.add_argument('--graspPointCloudPath', help='Grasp point cloud file path', required=True)
parser.add_argument('--objectPointCloudPath', help='Object point cloud file path', required=True)
parser.add_argument('--transformationFilePath', help='Transformation point cloud file path', required=True)

if __name__ == "__main__":
    start_time = time.time()
    args = vars(parser.parse_args())
    graspPointCloudPath = args['graspPointCloudPath']
    objectPointCloudPath = args['objectPointCloudPath']
    transformationFile = args['transformationFilePath']
    qty = Quality(graspPointCloudPath, objectPointCloudPath, transformationFile)
    
    #Pipeline
    graspPointCloud = qty.loadPointCloud(graspPointCloudPath)
    objectPointCloud = qty.loadPointCloud(objectPointCloudPath)
    filteredObjectPointCloud = qty.filterPointCloud(objectPointCloud)
    cm = qty.computeCenterPoint(filteredObjectPointCloud)
    pointCloudNormals = qty.computeNormals(filteredObjectPointCloud, cm)
    line = qty.extractGraspNumber(graspPointCloudPath)
    transformation = qty.returnTransformation(transformationFile, line)
    [transformedGraspPointCloud, bbox] = qty.getHandPCTransformation(graspPointCloud, transformation)
    [convex_hull, objectCroppedPointCloud] = qty.computeQTpoints(transformedGraspPointCloud, filteredObjectPointCloud, bbox, line)
    qty.visualizeGraspVTK(filteredObjectPointCloud, transformedGraspPointCloud, objectCroppedPointCloud, line)
    qty.visualiazeGraspO3D(transformedGraspPointCloud, filteredObjectPointCloud, objectCroppedPointCloud, bbox, convex_hull)

    line_str = str(line)
    logger.info("Time to compute qualities for grasp " + line_str + " took: --- %s seconds ---" % (time.time() - start_time))