#!/usr/bin/env python
import numpy as np
from numpy import dot

class ContactPoint:
    def __init__(self, p, n, id):
        self.p = p
        self.n = n
        self.id = id

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def getRot(fromm, to):
        fN = Quaternion.normalize(fromm)
        tN = Quaternion.normalize(to)
        d = np.dot(fN, tN)
        crossvec = np.cross(fN, tN)
        crosslen = np.linalg.norm(crossvec)
        q = Quaternion
        vector1 = [1.0, 0.0, 0.0]

        if(crosslen == 0.0):
            if(d < 0.0):
                t = np.cross(fN, vector1)
                
    
    def normalize(vector):
        norm = np.linalg.norm(vector)
        if norm == 0:
            return vector
        return vector / norm

class TriangleFace6D:
    def __init__(self, id, normal, verts, distNormZero, distNormCenter, distPlaneZero, distPlaneCenter, offset):
        self.id = id
        self.normal = normal
        self.verts = verts
        self.distNormZero = distNormZero
        self.distNormCenter = distNormCenter
        self.distPlaneZero = distPlaneZero
        self.distPlaneCenter = distPlaneCenter
        self.offset = offset

class ConvexHull6D:
    def __init__(self, vertices, faces):
        self.vertices = vertices
        self.faces = faces
