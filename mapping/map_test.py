#!/usr/bin/env python3

import sys
import time
from datetime import datetime
import random
import numpy as np
from matplotlib import cm
import weakref
import carla
from opendrive2lanelet.opendriveparser.parser import parse_opendrive

if __name__ == "__main__":
    # test if the code works
    host = 'localhost'
    port = 2000
    
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    map_name = 'Town01'
    world = client.load_world(map_name)
    map = world.get_map()
    open_drive = parse_opendrive(map.to_opendrive())

    waypoint_list = []

    for road in open_drive.roads:
        way_point_start = map.get_waypoint_xodr(road.id, -1, 0)
        if way_point_start is not None:
            waypoint_list.append(way_point_start)
            waypoint_list.extend(way_point_start.next_until_lane_end(8))
    
    print(len(waypoint_list))
    for waypoint in waypoint_list:
        world.debug.draw_point(
                waypoint.transform.location ,
                life_time=1200)


    
    
    