import open3d as o3d
import numpy as np
from numpy.linalg import inv
import array
import time, math, os, sys, argparse, re

parser = argparse.ArgumentParser(description='Quality measures for transferring objects')
parser.add_argument('--graspPointCloudPath', help='Grasp point cloud file path', required=True)
parser.add_argument('--objectPointCloudPath', help='Object point cloud file path', required=True)
parser.add_argument('--transformationFilePath', help='Transformation point cloud file path', required=True)

def loadPointCloud(pointCloud):
    pc = o3d.io.read_point_cloud(pointCloud)
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
    pointCloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    #o3d.geometry.orient_normals_to_align_with_direction(pointCloud, orientation_reference=array([cm[0], cm[1], cm[2]]))
    #o3d.visualization.draw_geometries([pointCloud], point_show_normal=True)
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
    transformedGraspPointCloud = o3d.geometry.Geometry3D.transform(graspPointCloud, Tinv)
    bbox = o3d.geometry.OrientedBoundingBox.get_oriented_bounding_box(transformedGraspPointCloud)
    return transformedGraspPointCloud, bbox

def computeQTpoints(transformedGraspPointCloud, objectPointCloud, bbox):
    maxBound = o3d.geometry.PointCloud.get_max_bound(transformedGraspPointCloud)
    minBound = o3d.geometry.PointCloud.get_min_bound(transformedGraspPointCloud)
    
    o3d.geometry.crop_point_cloud(transformedGraspPointCloud, minBound, maxBound)
    o3d.visualization.draw_geometries([transformedGraspPointCloud, objectPointCloud, bbox], point_show_normal=False)

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
    computeQTpoints(transformedGraspPointCloud, filteredObjectPointCloud, bbox)
