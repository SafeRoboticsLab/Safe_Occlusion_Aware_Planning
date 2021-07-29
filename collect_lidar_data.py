#!/usr/bin/env python3


import numpy as np
import matplotlib
matplotlib.use('TKAgg')#conflict with opencv due to Qt issue
import matplotlib.pyplot as plt
from mapping import OccupancyMap
import pickle








if __name__ == "__main__":

    filename = "data/lidar/Town01"
    infile = open(filename+'.pkl','rb')
    pc_list = pickle.load(infile)
    infile.close()

    occupancy_map = OccupancyMap(resolution=0.5)

    print(len(pc_list))
    pc, pose = pc_list[500]
    print(pc.shape)
    occupancy_map.insert_point_cloud(pc, [0.0 for _ in range(6)])
    occupancy_map.visualize()

    
    