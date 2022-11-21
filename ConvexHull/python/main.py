#!/usr/bin/env python
import argparse, time
import logging, coloredlogs
import multiprocessing as multip
from concurrent.futures import ProcessPoolExecutor
from mvbb import Quality
from dataGrasp import DataGrasp
from inOut import InOut as inout

LOG_LEVEL = logging.DEBUG
logger = logging.getLogger('Quality Measures')
coloredlogs.install(level=LOG_LEVEL, logger=logger)

parser = argparse.ArgumentParser(description='Quality measures for transferring objects')
parser.add_argument('--graspPointCloudPath', help='Grasp point cloud file path', required=True)
parser.add_argument('--objectPointCloudPath', help='Object point cloud file path', required=True)
parser.add_argument('--transformationFilePath', help='Transformation point cloud file path', required=True)

args = vars(parser.parse_args())
graspPointCloudPath = args['graspPointCloudPath']
objectPointCloudPath = args['objectPointCloudPath']
transformationFile = args['transformationFilePath']

# CLASSES 
qty = Quality(graspPointCloudPath, objectPointCloudPath, transformationFile)
data = DataGrasp(graspPointCloudPath)
io = inout(graspPointCloudPath, objectPointCloudPath, transformationFile)

def computeQualities(index):
    #Pipeline
    newGraspPointCloudPath = inout.changeGraspPointcloud(graspPointCloudPath, index)
    graspNumber = data.getGraspNumber(newGraspPointCloudPath)
    graspPointCloud = io.loadPointCloud(newGraspPointCloudPath)
    objectPointCloud = io.loadPointCloud(objectPointCloudPath)
    filteredObjectPointCloud = qty.filterPointCloud(objectPointCloud)
    cm = qty.computeCenterPoint(filteredObjectPointCloud)
    pointCloudNormals = qty.computeNormals(filteredObjectPointCloud, cm)
    transformation = qty.returnTransformation(transformationFile, graspNumber)
    [transformedGraspPointCloud, bbox] = qty.getHandPCTransformation(graspPointCloud, transformation)
    [convex_hull, objectCroppedPointCloud] = qty.computeQTpoints(transformedGraspPointCloud, filteredObjectPointCloud, bbox, graspNumber)
    # qty.visualizeGraspVTK(filteredObjectPointCloud, transformedGraspPointCloud, objectCroppedPointCloud, graspNumber)
    # qty.visualiazeGraspO3D(transformedGraspPointCloud, filteredObjectPointCloud, objectCroppedPointCloud, bbox, convex_hull)

    # graspNumber_str = str(graspNumber)
    # print("Grasp number is:", graspNumber_str)
    # logger.info("Time to compute qualities for grasp " + graspNumber_str + " took: --- %s seconds ---" % (time.time() - start_time))

def main():
    cpuAmount = multip.cpu_count()
    executor = ProcessPoolExecutor(max_workers = cpuAmount)
    logger.info("Multithreading in " + str(cpuAmount) + " threads")

    #Loop for the main pipeline
    # for index in range(1,31):
    #     computeQualities(index)

    #Multithreading
    with ProcessPoolExecutor(max_workers=cpuAmount) as executor:
        executor.map(computeQualities, range(31))


if __name__ == "__main__":
    start_time = time.time()
    logger.info("Starting...")
    main()
    logger.info("Time to compute qualities took: --- %s seconds ---" % (time.time() - start_time))