from open3d import *
import numpy as np
from numpy.linalg import inv
import array
import logging, coloredlogs
import time, math, os, sys, argparse, re
from quality import Quality

LOG_LEVEL = logging.DEBUG
logger = logging.getLogger('Quality Measures')
coloredlogs.install(level=LOG_LEVEL, logger=logger)

parser = argparse.ArgumentParser(description='Quality measures for transferring objects')
parser.add_argument('--graspPointCloudPath', help='Grasp point cloud file path', required=True)
parser.add_argument('--objectPointCloudPath', help='Object point cloud file path', required=True)
parser.add_argument('--transformationFilePath', help='Transformation point cloud file path', required=True)

def loadPointCloud(pointCloud):
    pc = io.read_point_cloud(pointCloud, format='auto', remove_nan_points=True, remove_infinite_points=True, print_progress=True)
    return pc

def filterPointCloud(objectpointCloud):
    points = np.asarray(objectpointCloud.points)
    if(len(points) > 900000):
        filteredPointCloud = objectpointCloud.voxel_down_sample(voxel_size=0.05)
        print("Object point cloud filtered")
        return filteredPointCloud
    else:
        return objectpointCloud

def computeCenterPoint(pointCloud):
    samplePoints = np.asarray(pointCloud.points)
    accumulative = [0.0 ,0.0, 0.0]
    for point in samplePoints:
        accumulative += point
    return accumulative / len(samplePoints)

def computeNormals(pointCloud, centerPoint):
    pointCloud.estimate_normals(search_param=geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # geometry.orient_normals_to_align_with_direction(pointCloud, orientation_reference=array([cm[0], cm[1], cm[2]]))
    # visualization.draw_geometries([pointCloud], point_show_normal=True)
    return pointCloud

def extractGraspNumber(graspPointCloudPath):
    number = re.findall('\d+', graspPointCloudPath )
    number = int(number[0])
    return number

def returnTransformation(transformationsFilePath, line):
    file = open(transformationsFilePath)
    all_lines = file.readlines()
    row = all_lines[line - 1]
    x = row.split(", ")
    transformation = np.array([[float(x[0]), float(x[1]), float(x[2]), float(x[3])], 
                               [float(x[4]), float(x[5]), float(x[6]), float(x[7])],
                               [float(x[8]), float(x[9]), float(x[10]), float(x[11])],
                               [0, 0, 0, 1]])
    return transformation

def getHandPCTransformation(graspPointCloud, transformation):
    normaPC = graspPointCloud
    Tinv = np.linalg.inv(transformation)
    transformedGraspPointCloud = geometry.Geometry3D.transform(graspPointCloud, Tinv)
    bbox = geometry.OrientedBoundingBox.get_oriented_bounding_box(transformedGraspPointCloud)
    return transformedGraspPointCloud, bbox

def computeQTpoints(transformedGraspPointCloud, objectPointCloud, bbox, line):
    maxBound = geometry.PointCloud.get_max_bound(transformedGraspPointCloud)
    minBound = geometry.PointCloud.get_min_bound(transformedGraspPointCloud)
    hull, _ = transformedGraspPointCloud.compute_convex_hull()
    convex_hull = geometry.LineSet.create_from_triangle_mesh(hull)
    objectCroppedPointCloud = geometry.PointCloud.crop(objectPointCloud, bbox)
    partialInPoints = len(np.asarray(objectCroppedPointCloud.points))
    totalPoints = len(np.asarray(objectPointCloud.points))
    QTpoints = (totalPoints - partialInPoints) / totalPoints
    logger.info("QMTpoints for grasp %d is %.4f" % (line, QTpoints))
    return convex_hull, objectCroppedPointCloud

def getPointCloudArea(objectPointCloud):
    objectPointCloud.estimate_normals()
    distances = objectPointCloud.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 10 * avg_dist   
    mesh1 = geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(objectPointCloud, utility.DoubleVector([radius, radius * 2]))
    # mesh2 = geometry.TriangleMesh.create_from_point_cloud_alpha_shape(objectPointCloud, 0.9)
    # visualization.draw_geometries([mesh1])
    
def visualize(transformedGraspPointCloud, objectPointCloud, objectCroppedPointCloud, bbox, convex_hull):
    transformedGraspPointCloud.paint_uniform_color([1, 0, 0])
    objectPointCloud.paint_uniform_color([0, 1, 0])
    objectCroppedPointCloud.paint_uniform_color([0, 0, 1])
    visualization.draw_geometries([transformedGraspPointCloud, objectPointCloud, objectCroppedPointCloud, bbox], point_show_normal=False,)

if __name__ == "__main__":
    args = vars(parser.parse_args())
    graspPointCloudPath = args['graspPointCloudPath']
    objectPointCloudPath = args['objectPointCloudPath']
    transformationFile = args['transformationFilePath']
    #Pipeline
    graspPointCloud = loadPointCloud(graspPointCloudPath)
    objectPointCloud = loadPointCloud(objectPointCloudPath)
    filteredObjectPointCloud = filterPointCloud(objectPointCloud)
    cm = computeCenterPoint(filteredObjectPointCloud)
    pointCloudNormals = computeNormals(filteredObjectPointCloud, cm)
    line = extractGraspNumber(graspPointCloudPath)
    transformation = returnTransformation(transformationFile, line)
    [transformedGraspPointCloud, bbox] = getHandPCTransformation(graspPointCloud, transformation)
    [convex_hull, objectCroppedPointCloud] = computeQTpoints(transformedGraspPointCloud, filteredObjectPointCloud, bbox, line)
    # getPointCloudArea(filteredObjectPointCloud)
    visualize(transformedGraspPointCloud, filteredObjectPointCloud, objectCroppedPointCloud, bbox, convex_hull)