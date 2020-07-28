#!/usr/bin/env python
from open3d import *
import numpy as np
from numpy.linalg import inv
import logging, coloredlogs
import math, os, sys, re, array, vtk
import pyvista as pv

LOG_LEVEL = logging.DEBUG
logger = logging.getLogger('Quality Measures')
coloredlogs.install(level=LOG_LEVEL, logger=logger)

class Quality:

    def __init__(self, graspPointCloudPath, objectPointCloudPath, transformationFile):
        self.graspPointCloudPath = graspPointCloudPath
        self.objectPointCloudPath = objectPointCloudPath
        self.transformationFile = transformationFile


    def loadPointCloud(self, pointCloud):
        pc = io.read_point_cloud(pointCloud, format='auto', remove_nan_points=True, remove_infinite_points=True, print_progress=True)
        return pc


    def filterPointCloud(self, objectpointCloud):
        points = np.asarray(objectpointCloud.points)
        if(len(points) > 900000):
            filteredPointCloud = objectpointCloud.voxel_down_sample(voxel_size=0.05)
            print("Object point cloud filtered")
            return filteredPointCloud
        else:
            return objectpointCloud


    def computeCenterPoint(self, pointCloud):
        samplePoints = np.asarray(pointCloud.points)
        accumulative = [0.0, 0.0, 0.0]
        for point in samplePoints:
            accumulative += point
            
        cp = accumulative / len(samplePoints)
        return cp


    def computeNormals(self, pointCloud, centerPoint):
        pointCloud.estimate_normals(search_param=geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        # geometry.orient_normals_to_align_with_direction(pointCloud, orientation_reference=array([cm[0], cm[1], cm[2]]))
        return pointCloud


    def extractGraspNumber(self, graspPointCloudPath):
        number = re.findall('\d+', graspPointCloudPath)
        number = int(number[0])
        return number


    def returnTransformation(self, transformationsFilePath, line):
        file = open(transformationsFilePath)
        all_lines = file.readlines()
        row = all_lines[line - 1]
        x = row.split(", ")
        transformation = np.array([[float(x[0]), float(x[1]), float(x[2]), float(x[3])], 
                                   [float(x[4]), float(x[5]), float(x[6]), float(x[7])],
                                   [float(x[8]), float(x[9]), float(x[10]), float(x[11])],
                                   [0, 0, 0, 1]])
        return transformation


    def getHandPCTransformation(self, graspPointCloud, transformation):
        normaPC = graspPointCloud
        Tinv = np.linalg.inv(transformation)
        transformedGraspPointCloud = geometry.Geometry3D.transform(graspPointCloud, Tinv)
        bbox = geometry.OrientedBoundingBox.get_oriented_bounding_box(transformedGraspPointCloud)
        return transformedGraspPointCloud, bbox


    def computeQTpoints(self, transformedGraspPointCloud, objectPointCloud, bbox, line):
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


    def visualizeGraspVTK(self, objectPointCloud, graspPointCloud, partialObjectPointCloud, line):
        line_str = str(line)  
        objectPTS = np.asarray(objectPointCloud.points)
        object = pv.PolyData(objectPTS)    
        objectMesh = object.delaunay_3d(alpha=0.022)
        objectWires = objectMesh.compute_cell_sizes(length=False, area=False, volume=False).extract_all_edges()

        graspPTS = np.asarray(graspPointCloud.points)
        grasp = pv.PolyData(graspPTS)
        graspMesh = grasp.delaunay_3d(alpha=0.0115)
        graspWires = graspMesh.compute_cell_sizes(length=False, area=False, volume=True).extract_all_edges()   

        partialPTS = np.asarray(partialObjectPointCloud.points)
        partial = pv.PolyData(partialPTS)
        partialMesh = partial.delaunay_3d(alpha=0.03)
        partialWires = partialMesh.compute_cell_sizes(length=True, area=False, volume=False).extract_all_edges() 
    
        plotter = pv.Plotter(polygon_smoothing=True, 
                             border_width=10.0,
                             point_smoothing=True, 
                             border=True, 
                             border_color='white'
                            )
        plotter.add_mesh(objectMesh, color='green')
        plotter.add_mesh(graspMesh, color='red')
        plotter.add_mesh(partialMesh, color='blue')
        plotter.show_bounds(grid='front', location='outer', all_edges=True)
        plotter.show(title="GRASP " + line_str, full_screen=False) 


    def visualiazeGraspO3D(self, transformedGraspPointCloud, objectPointCloud, objectCroppedPointCloud, bbox, convex_hull):
        transformedGraspPointCloud.paint_uniform_color([1, 0, 0])
        objectPointCloud.paint_uniform_color([0, 1, 0])
        objectCroppedPointCloud.paint_uniform_color([0, 0, 1])
        visualization.draw_geometries([transformedGraspPointCloud, objectPointCloud, objectCroppedPointCloud, bbox], point_show_normal=False)