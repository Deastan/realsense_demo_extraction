import sys
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
# import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
from inspect import currentframe, getframeinfo
import pickle

if __name__ == '__main__' :
    print("Ready")

    name = "demo_1"
    path_pickle = "pickle/" + name + ".pkl"
    with open(path_pickle, 'rb') as f:
        np_trajectory = pickle.load(f)

    vector_xy = [np_trajectory[0][1]- np_trajectory[0][0], np_trajectory[1][1]- np_trajectory[1][0]] 
    print(vector_xy)