#!/usr/bin/env python
import numpy as np
from numpy import dot
from numpy.linalg import eig
#import quaternion

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
        #Input: fromm = vector(3), to = vector(3) 
        fN = Quaternion.normalize(fromm)
        tN = Quaternion.normalize(to)
        d = np.dot(fN, tN)
        crossvec = np.cross(fN, tN)
        crosslen = np.linalg.norm(crossvec)
        q = Quaternion(0.0, 0.0, 0.0, 1.0)
        # Quaternion(x,y,z,w)
        if(crosslen == 0.0):
            if(d < 0.0):
                t = np.cross(fN, [1.0,0.0,0.0])
                if((t == np.linalg.norm(t)) == 0.0):
                    t = np.cross(fN,[0.0,1.0,0.0])

                t = np.linalg.norm(t)
                q.x = t[0]
                q.y = t[1]
                q.z = t[2]
                q.w = 0.0
        else:
            crossvec = np.linalg.norm(crossvec)
            crossvec *= np.sqrt(0.5 * np.fabs(1.0 - d))
            q.x = crossvec[0]
            q.y = crossvec[1]
            q.z = crossvec[2]
            q.w = np.sqrt(0.5 * np.fabs(1.0 + d))
        
        return q

    def quat2rotmatrix(Q): 
        #m = np.as_rotation_matrix(q)
        # Return 3x3 matrix 
        #return m
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
     
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
     
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
     
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])
                            
        return rot_matrix

    def eigen4f2quat(m):
        q = np.from_rotation_matrix(m, nororthogonal=True)
        return q
    
    def getAng(q):
        n = np.sqrt(pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2) + pow(q.w, 2))
        if(n < 1e-10):
            return 0.0
        n = 1.0 / n
        return (2.0 * np.arccos(q.w * n))
    
    def TPosition(pos, m):
        t = (pos.x, pos.y, pos.z, 1)
        t = m * t

    def normalize(vector):
        normal = np.linalg.norm(vector)
        if normal == 0:
            return vector
        return vector / normal

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
