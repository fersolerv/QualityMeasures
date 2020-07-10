from open3d import *
import numpy as np
from numpy.linalg import inv
import array
import time, math, os, sys, argparse, re

class Quality:

    def __init__(self, graspPointCloudPath, objectPointCloudPath, transformationFile):
        self.graspPointCloudPath = graspPointCloudPath
        self.objectPointCloudPath = objectPointCloudPath
        self.transformationFile = transformationFile