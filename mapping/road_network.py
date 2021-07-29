#!/usr/bin/env python3

import time 
from opendrive2lanelet.network import Network
from opendrive2lanelet.opendriveparser.elements.road import Road as OpenDriveRoad
from opendrive2lanelet.opendriveparser.elements.roadLanes import LaneSection as OpenDriveLaneSection
from opendrive2lanelet.conversion_lanelet_network import ConversionLaneletNetwork as LaneletLanework
from opendrive2lanelet.conversion_lanelet import ConversionLanelet as Lanelet

from opendrive2lanelet.utils import encode_road_section_lane_width_id, decode_road_section_lane_width_id
import numpy as np
import carla
from shapely.geometry import LineString, Point


# ignoring other type of road
LANE_FILTER = ["driving", "onRamp", "offRamp", "exit", "entry"]

class Road():
    def __init__(self, road: OpenDriveRoad, lanelet_network: LaneletLanework):
        self.opendrive_road = road # store it for the access of planeview
        self.road_id = self.opendrive_road.id
        self.length = road.length
        self.elevation = road.elevationProfile

        # list for lane section with key of OpenDrive LaneSection id [0,1,2,3,.....]
        self.lane_section_map = {}
        self.id_map = {} # search by lanelet id
        self.sPos_list = []  
        
        lane_section_list = self.opendrive_road.lanes.lane_sections # this is sorted by OpenDrive Praser

        for i, lane_section in enumerate(lane_section_list):
            if i < len(lane_section_list)-1:
                sec_length = lane_section_list[i+1].sPos - lane_section_list[i].sPos
            else: 
                sec_length = self.length - lane_section_list[i].sPos
            new_section = LaneSection(self, lane_section, sec_length, lanelet_network)
            self.lane_section_map[lane_section.idx] = new_section
            self.id_map.update(new_section.id_map)
            self.sPos_list.append(lane_section.sPos)

    def find_lane(self, section_id, lane_id, s):
        if section_id is None:
            section_id = self.find_section_id_by_s(s)
        if section_id in self.lane_section_map:
            lane_section = self.lane_section_map[section_id]
            return lane_section.find_lane(lane_id)
        else:
            return None
        
    def find_section_id_by_s(self, s):
        if s > self.length:
            return None
        idx = np.searchsorted(self.sPos_list, s, side='right')
        return self.lane_section_map[idx]
          
class LaneSection():
    def __init__(self, road, lane_section: OpenDriveLaneSection, length, lanelet_network: LaneletLanework):
        self.road= road
        self.section_id = lane_section.idx
        self.s_start = lane_section.sPos
        self.length = length
        
        # dict for lane with key of OpenDrive lane id [...,-2,-1,1,2,....]
        self.lane_map = {}
        self.id_map = {}
        for lane in lane_section.leftLanes+lane_section.rightLanes:
            # check if type match the filter
            if lane.type in LANE_FILTER:
                new_lane = Lane(self, lane.id, lanelet_network)
                self.lane_map[lane.id] = new_lane
                self.id_map[new_lane.lanelet_id] = new_lane

    def find_lane(self, lane_id):
        if lane_id in self.lane_map:
            return self.lane_map[lane_id]
        else:
            return None

class Lane():
    def __init__(self, section: LaneSection, lane_id, lanelet_network: LaneletLanework):
        self.lane_section = section
        self.lane_id = lane_id

        # record adjacent, cross, merge, and split 
        opendrive_id = encode_road_section_lane_width_id(self.lane_section.road.road_id, self.lane_section.section_id, lane_id, -1)
        self.lanelet = lanelet_network.find_lanelet_by_opendrive_id(opendrive_id)
        self.lanelet_id = self.lanelet.lanelet_id
        
        # observation state of waypoint:
        #    key is the relative 
        #    0 : Unobserved
        #    t : last time it is observed

        # key is relative dS w.r.t the start of lane section
        # waypoint_ds follows the direction of driving, so once if lane_id>0, map key is decrementally sorted
        self.waypoint_ds = self.lanelet.poses
        self.waypoint_z = self.lane_section.road.elevation.calc_z(self.waypoint_ds)
        self.waypoint_ds = np.round(self.waypoint_ds, 3)
        
        self.waypoint_map = {}
        
        self.adj_same = [] # list of lanlet_id of adjacent lane with the same direction of current lane
        self.adj_oppo = [] # list of lanlet_id of adjacent lane with the opposit direction of current lane


        if self.lanelet.adj_left is not None:
            if self.lanelet.adj_left_same_direction:
                self.adj_same.append(self.lanelet.adj_left)
            else:
                self.adj_oppo.append(self.lanelet.adj_left)

        if self.lanelet.adj_right is not None:
            if self.lanelet.adj_right_same_direction:
                self.adj_same.append(self.lanelet.adj_right)
            else:
                self.adj_oppo.append(self.lanelet.adj_right)

        self.merge = self.lanelet.merge
        self.split = self.lanelet.split
        self.cross = self.lanelet.cross
        self.predecessor = self.lanelet.predecessor
        self.successor = self.lanelet.successor

        self.cross_ds = {}
        self.merge_ds = {}
            
    @ property
    def road_id(self):
        return self.lane_section.road.road_id
    
    @ property
    def section_id(self):
        return self.lane_section.section_id
    
    @ property
    def length(self):
        return self.lane_section.length

    @ property
    def center_vertices(self):
        vertices_raw = self.lanelet.center_vertices 
        return np.array([vertices_raw[:,0], -vertices_raw[:,1], self.waypoint_z]).T 
    
    @ property
    def left_vertices(self):
        vertices_raw = self.lanelet.left_vertices 
        return np.array([vertices_raw[:,0], -vertices_raw[:,1], self.waypoint_z]).T 

    @ property
    def right_vertices(self):
        vertices_raw = self.lanelet.right_vertices 
        return np.array([vertices_raw[:,0], -vertices_raw[:,1], self.waypoint_z]).T 

    def road_bound_vertices(self, ds_L=None, ds_U=None):
        """
            in the order of traffic flow, get left and right vertices of the road between ds_L and ds_U
            if lane_id < 0. ds_L < ds_U
            if lane_id > 0. ds_L > ds_U
        """

        if ds_L is None:
            idx_L = 0
        else:
            idx_L = np.argmin(np.abs(self.waypoint_ds - ds_L))
        
        if ds_U is None:
            idx_U = self.waypoint_ds.shape[0]-1
        else:
            idx_U = np.argmin(np.abs(self.waypoint_ds - ds_U))

        #print(ds_L, ds_U, idx_L, idx_U, self.waypoint_ds[0], self.waypoint_ds[-1])

        # sanity check
        if idx_U<idx_L:
            print(ds_L, ds_U, idx_L, idx_U)
            raise ValueError("idx_L is larger than idx_U")
        
        return self.right_vertices[idx_L:idx_U+1, :],  self.left_vertices[idx_L:idx_U+1, :]

    def update(self, ds, status, debug=None):
        if isinstance(ds, list):
            # update all keys within the range of ds
            ds_range = np.sort(ds).tolist() # incremental order
            key_to_update = self.waypoint_ds[(self.waypoint_ds>=ds_range[0])& (self.waypoint_ds<=ds_range[1])]
            center_vertices = self.center_vertices
            for key in key_to_update:
                self.waypoint_map[key] = status
                if debug is not None:
                    idx = np.where(self.waypoint_ds == key)[0][0]
                    
                    location = carla.Location(x = center_vertices[idx, 0], y=center_vertices[idx, 1], z=center_vertices[idx, 2]+1)
                    debug.draw_string(location, "X")

        else:
            idx = np.abs(self.waypoint_ds-ds).argmin()
            self.waypoint_map[self.waypoint_ds[idx]] = status
            if debug is not None:
                    center_vertices = self.center_vertices
                    location = carla.Location(x = center_vertices[idx, 0], y=center_vertices[idx, 1], z=center_vertices[idx, 2]+1)
                    debug.draw_string(location, "X")

    def get_cross_ds(self, cross_id, cross_lane):
        if cross_id not in self.cross:
            return None
        
        if cross_id not in self.cross_ds:
            # first time search
            # get left and right lane vertice of cross lane.
            # find intersectio with current lane's center vertices

            # construct center line
            ego_center_shell = []
            for vertice in self.center_vertices:
                ego_center_shell.append((vertice[0], vertice[1]))

            cross_left_shell = []
            for vertice in cross_lane.left_vertices:
                cross_left_shell.append((vertice[0], vertice[1]))

            cross_right_shell = []
            for vertice in cross_lane.right_vertices:
                cross_right_shell.append((vertice[0], vertice[1]))

            ego_center = LineString(ego_center_shell)
            ego_left = LineString(cross_left_shell)
            ego_right = LineString(cross_right_shell)

            
            def find_closest_ds(intersection: Point):
                x, y = intersection.coords.xy
                intersection_np = np.array([x[0],y[0]])
                center_np = self.center_vertices[:,:2] #nx2
                idx_close = np.linalg.norm(center_np - intersection_np, axis=1).argmin() # norm in each row
                return self.waypoint_ds[idx_close]


            ds_left = find_closest_ds(ego_center.intersection(ego_left))
            ds_right = find_closest_ds(ego_center.intersection(ego_right))

            if self.lane_id<0:
                self.cross_ds[cross_id] = np.sort([ds_left, ds_right])
            else:
                self.cross_ds[cross_id] = np.flip(np.sort([ds_left, ds_right]))
        
        return self.cross_ds[cross_id]



    def delta_s_from_begin(self, ds):
        """
        Given ds in this lane, calculate the absolute distance between ds to the begin of the lane.
        if lane_id<0, the lane begin at ds=0,
        if lane_id>0, the lane begin at ds=length 
        """
        return np.abs( (self.lane_id>0)*self.length - ds)
    
    def delta_s_from_end(self, ds):
        """
        Given ds in this lane, calculate the absolute distance between ds to the end of the lane.
        if lane_id<0, the lane ends at ds=length
        if lane_id>0, the lane ends at ds=0 
        """
        return np.abs( (self.lane_id<0)*self.length - ds)



class RoadNetwork():
    """
    class that maintain a network regarding Carla world
    it store the successor, predecessor and conflicts of each lane section
    represented by both Lanelet and OpenDrive format
    """

    def __init__(self, client, ds = 0.5):
        # parameters
        self.client = client
        self.ds = ds

        self.road_map = {} # map that can be searched through opendrive road_id, section_id, lane_id
        self.id_map  = {} # map that can be searched through lanelet id

        self._construct_network()

    def _construct_network(self):
        # opendrive2lanelet package https://opendrive2lanelet.readthedocs.io/en/latest/
        # prase the Opendrive file and convert to 

        road_network = Network()
        road_network.load_opendrive_str(self.client.map.to_opendrive(), self.ds)
        lanelet_network = road_network.export_lanelet_network(filter_types=LANE_FILTER, concatenate=False, precision=self.ds)

        for road in road_network.opendrive.roads:
            new_road = Road(road, lanelet_network)
            self.road_map[road.id] = new_road
            self.id_map.update(new_road.id_map)
        
    def find_lane_id(self, id) -> Lane:
        if id in self.id_map:
            return self.id_map[id]
        else:
            return None
        
    def find_lane(self, road_id, section_id, lane_id, s=None) -> Lane:
        """
            Given the road_id, lane_id, and either section_id or s
            return the corresponding Lane class
        
        """

        if road_id in self.road_map:
            return self.road_map[road_id].find_lane(section_id, lane_id, s)
        else:
            return None

    def debug_plot(self, road_id, section_id, lane_id):
        lane = self.find_lane(road_id, section_id, lane_id)
        if lane is not None:
            for center_vertice in lane.center_vertices():
                #print(center_vertice)
                location = carla.Location(x = center_vertice[0], y=center_vertice[1], z=center_vertice[2]+1)
                self.client.world.debug.draw_string(location, "X")
                self.client.tick()
                self.client.render()
                time.sleep(0.1)
        else:
            print("None")

    def update_state_batch(self, t, road_id, section_id, lane_id, ds, s_range, from_end=False, ds_global = False):
        """
            update a batch of consecutive waypoints centered at (road_id, section_id, lane_id, ds)
            where ds is the realtive S w.r.t to the section start. 
            if From the end is true, then ds is relative s w.r.t to the end of the section
            s_range = [s_before, s_after] 
            if lane_id < 0
                update [ds-s_before, ds+s_after]
            if lane_id>0
                update [ds+s_before, ds-s_after]
        """
        lane = self.find_lane(road_id, section_id, lane_id)
        if ds_global:
            if not from_end:
                ds = ds - lane.lane_section.s_start
            else:
                ds = ds - (lane.lane_section.s_start+lane.lane_section.length)

        if from_end:
            ds = lane.lane_section.length - ds
        s0 = ds+np.sign(lane_id)*s_range[0]
        s1 = ds-np.sign(lane_id)*s_range[1]
        lane.update([max(0, s0), min(s1, lane.length)], (t, True))#, debug = self.client.world.debug)

        # if the current batch cover other lanes, update them recursively
        if s0 < 0:
            if lane_id <0: # begin at s=0. update its predecessor
                for predecessor in lane.predecessor:
                    new_lane = self.find_lane_id(predecessor)
                    if new_lane.lane_id<0: # predecessor's end connect current lane's begining
                        self.update_state_batch(t, new_lane.road_id, new_lane.section_id, new_lane.lane_id, 0, [-s0, 0], True)
                    else:
                        self.update_state_batch(t, new_lane.road_id, new_lane.section_id, new_lane.lane_id, 0, [0, -s0], False)
            else: # end at s=0. update its successor
                for successor in lane.successor:
                    new_lane = self.find_lane_id(successor)                
                    if new_lane.lane_id<0: # current lane's begin connect with the begin of successor
                        self.update_state_batch(t, new_lane.road_id, new_lane.section_id, new_lane.lane_id, 0, [0, -s0], False)
                    else: # current lane's begin connect with the end of successor
                        self.update_state_batch(t, new_lane.road_id, new_lane.section_id, new_lane.lane_id, 0, [-s0, 0], True)


        if s1 > lane.length:
            if lane_id>0: # begin at s=length. update its predecessor
                for predecessor in lane.predecessor:
                    new_lane = self.find_lane_id(predecessor)
                    if new_lane.lane_id<0: # predecessor's end connect current lane's begining
                        self.update_state_batch(t, new_lane.road_id, new_lane.section_id, new_lane.lane_id, 0, [s1-lane.length, 0], True)
                    else:
                        self.update_state_batch(t, new_lane.road_id, new_lane.section_id, new_lane.lane_id, 0, [0, s1-lane.length], False)
            else: # end at s=length. update its successor
                for successor in lane.successor:
                    new_lane = self.find_lane_id(successor)
                    if new_lane.lane_id<0: # current lane's begin connect with the begin of successor
                        self.update_state_batch(t, new_lane.road_id, new_lane.section_id, new_lane.lane_id, 0, [0, s1-lane.length], False)
                    else: # current lane's begin connect with the end of successor
                        self.update_state_batch(t, new_lane.road_id, new_lane.section_id, new_lane.lane_id, 0, [s1-lane.length, 0], True)



            



        

    

