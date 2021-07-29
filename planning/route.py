from ast import fix_missing_locations
from client import SynchronousClient
from mapping import RoadNetwork
import numpy as np
from queue import Queue


class RouteSection():
    def __init__(self, id, successor, lane_change, p_start, road_network: RoadNetwork, client: SynchronousClient):
        """ 
            This class record a nominal lane, and allowable lateral displacement
        """
        self.road_id = id[0]
        self.section_id = id[1]
        self.nominal_lane_id = id[2]
        self.p_start = p_start

        cur_lane = road_network.find_lane(*id)
        self.nominal_lanelet_id = cur_lane.lanelet_id
        self.successor = successor # key (road_id, section_id) of the successor route lane
        self.length = cur_lane.length
       
        
        def get_lane_width(lane):
            '''
            given a Roadnetwork.Lane object, find the width of the lane from the OXDR infromation
            '''
            waypoint_1 = client.map.get_waypoint_xodr(lane.road_id, lane.lane_id, lane.lane_section.s_start)
            waypoint_2 = client.map.get_waypoint_xodr(lane.road_id, lane.lane_id, lane.lane_section.s_start+lane.length-0.5)

            return max(waypoint_1.lane_width, waypoint_2.lane_width)
        
        
        ego_width = get_lane_width(cur_lane)
        # bound lateral direction for diriving lane with the same direction of current lane 
        # l w.r.t to the center of ego lane. decrease to the left, incresea to the right
        self.l_oppo = None
        l_same = [-ego_width/2, ego_width/2]
        l_left = [-ego_width/2]
        l_right = [ego_width/2]
        self.id_list = [self.nominal_lanelet_id]
        self.id_same = [self.nominal_lanelet_id]
        self.id_oppo = None
        
        if lane_change:
            # search toward right
            i=1
            while True:
                cur_lane = road_network.find_lane(self.road_id, self.section_id, 
                                            self.nominal_lane_id+np.sign(self.nominal_lane_id)*i)
                if cur_lane is None:
                    break
                width = get_lane_width(cur_lane)
                l_left.append(l_right[-1])
                l_right.append(l_right[-1]+width)
                l_same[-1]+=width
                self.id_same.append(cur_lane.lanelet_id)
                self.id_list.append(cur_lane.lanelet_id)
                i+=1
            # search toward left
            i=1
            while True:
                new_lane_id = self.nominal_lane_id-np.sign(self.nominal_lane_id)*i
                if new_lane_id == 0:
                    i+=1
                else:
                    cur_lane = road_network.find_lane(self.road_id, self.section_id, new_lane_id)
                    if cur_lane is None:
                        break
                    width = get_lane_width(cur_lane)
                    l_right.insert(0, l_left[0])
                    l_left.insert(0,l_left[-1]-width)
                    self.id_list.insert(0,cur_lane.lanelet_id)
                    if np.sign(new_lane_id) == np.sign(self.nominal_lane_id):    
                        l_same[0]-=width
                        self.id_same.insert(0, cur_lane.lanelet_id)
                        i+=1
                    else:
                        self.l_oppo = np.array([l_left[0]-width, l_left[0]])
                        self.id_oppo = cur_lane.lanelet_id
                        break

        self.l_same = np.array(l_same)
        self.l_left = np.array(l_left)
        self.l_right = np.array(l_right)

    def get_neighbor(self, l, right_side = True):
        idx = np.searchsorted(self.l_left, l, side = 'right')-1
        neighbor = None
        if right_side:
            if idx<(len(self.id_list)-1):
                neighbor = (self.id_list[idx+1], self.l_left[idx+1], self.l_right[idx+1])
        else:
            if idx > 0:
                neighbor = (self.id_list[idx-1], self.l_left[idx-1], self.l_right[idx-1])

        return neighbor
    
    def get_boundary(self, lanelet_idx):
        idx = np.where(np.array(self.id_list) == lanelet_idx)[0][0]
        return self.l_left[idx], self.l_right[idx]
       
    def is_same(self, lanelet_id):
        # given a lanelet id, determine if it is in the allowable lane of the route with the same direction
        return lanelet_id in self.id_same
    
    def is_oppo(self, lanelet_id):
        # given a lanelet id, determine if it is in the allowable lane of the route with opposite direction 
        return lanelet_id in self.id_oppo

    def l_to_id(self, l, width):
        """
        Given a l, return 1) the lanelet id of that lateral displacement
                 2) Boolean to indicates if this lane has the same direction of nominal lane
        """
        l_left = l - width/2
        l_right = l + width/2
        # laying outside of allowable region
        if l_right>self.l_right[-1]:
            return None, None

        if l_left<self.l_left[0]:
            return None, None

        idx_left = np.searchsorted(self.l_left, l_left, side = 'right')-1
        idx_right = np.searchsorted(self.l_right, l_right, side = 'left')
    
        lanelet_left =  self.id_list[idx_left]
        lanelet_right = self.id_list[idx_right]
        
        if lanelet_left == lanelet_right:
            return [lanelet_left], [lanelet_left in self.id_same]
        else:
            return [lanelet_left, lanelet_right], [lanelet_left in self.id_same, lanelet_right in self.id_same]
    
class Route():
    def __init__(self, client:SynchronousClient, road_network: RoadNetwork, route):
        """
        
        The route planner provides a nomianl route consisting a list of nomianl lane ids in the form of (road_id, section_id, lane_id)
        In addition, it also indicates if we have to drive on this specific lane or we can use the adjunct lanes 
        that are all adjunct lanes with same direction of nominal lanes and closest adjunct lanes with opposite traffic direction


        
        route: a list of (road_id, section_id, lane_id, Boolean) for ego's future path
                        Ture means lane change allowed 
                        False means lane change not allowed 
        """

        self.client = client
        self.road_network = road_network
        self.route = route
        self.route_map = {}
        self.total_length = 0
        self.p_start = []
        self.allowable_lanelet = []
        for i, lane in enumerate(route):
            # key : (road_id, section_id)
            # item: (RouteSection obj)
            if i+1 < len(route):
                self.route_map[(lane[0], lane[1])] = RouteSection(lane[:3], route[i+1][:2], lane[3], self.total_length, self.road_network, self.client)
            else:
                self.route_map[(lane[0], lane[1])] = RouteSection(lane[:3], None, lane[3], self.total_length, self.road_network, self.client)
            self.allowable_lanelet.extend(self.route_map[(lane[0], lane[1])].id_list)
            self.p_start.append(self.total_length)
            self.total_length += self.route_map[(lane[0], lane[1])].length
            
        
     
    def in_ego_path(self, lanelet_id):
        """
        Given a lanelet id of nominal lane, check if this lane is the neighbor of route lane with same direction and allowable to drive
        """
        # lane = self.road_network.find_lane_id(nominal_lanelet_id)
        # road_id = lane.road_id
        # section_id = lane.section_id

        # if (road_id, section_id) in self.route_map:
        #     route_section = self.route_map[(road_id, section_id)]
        #     return route_section.is_same(nominal_lanelet_id)
        # return False
        return lanelet_id in self.allowable_lanelet
   
    def find_lanelet_id(self, nominal_lanelet_id, l, width):
        """
        Given a lanelet id of nominal lane, lateral displacement, and width of ego vehicle
        generate a list of lanelet id that the ego vehicle is covering
        """
        lane = self.road_network.find_lane_id(nominal_lanelet_id)
        road_id = lane.road_id
        section_id = lane.section_id
        if (road_id, section_id) in self.route_map:
            return self.route_map[(road_id, section_id)].l_to_id(l, width)
        return None, None

    def get_route_section(self, p) -> RouteSection:
        idx = np.searchsorted(self.p_start, p, side='right') - 1
        section_id = self.route[idx]
        return self.route_map[(section_id[0], section_id[1])]

    def find_nominal_lane(self, p):
        """
        Given the distance since the begin of this section, and lateral displacement, 
        find the nominal laneletid and ds
        """
        route_section = self.get_route_section(p = p)
        
        p_from_sec_start = p - route_section.p_start
        ds = np.abs((route_section.nominal_lane_id>0)*route_section.length - p_from_sec_start)

        return route_section.nominal_lanelet_id, ds

    def find_global_position(self, p, l):
        """
        Given the distance since the begin of this section, and lateral displacement, find the global position
        """
        route_section = self.get_route_section(p = p)
        p_from_sec_start = p - route_section.p_start
        
        lanelet_id = route_section.nominal_lanelet_id
        lane = self.road_network.find_lane_id(lanelet_id)
        
        s = np.abs((route_section.nominal_lane_id>0)*lane.length - p_from_sec_start) + lane.lane_section.s_start

        waypoint_center = self.client.map.get_waypoint_xodr(lane.road_id, lane.lane_id, s)

        transfrom = waypoint_center.transform

        T_v2w = np.array(transfrom.get_matrix())
        pos = np.matmul(T_v2w, np.array([0, l, 0, 1]))

        transfrom.location.x = pos[0]
        transfrom.location.y = pos[1]
        transfrom.location.z = pos[2]+0.1

        return transfrom





