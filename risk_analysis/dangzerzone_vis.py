from mapping import RoadNetwork
from planning import Route
from queue import Queue
import numpy as np

class DangerZonePlot():
    def __init__(self):
        self.type = None # ture for fwd, false for bwd
        self.start_id = None
        self.start_ds = None
        self.length = None
        self.end_id = []
        self.end_ds = []
        self.coverage_list = []
        self.polygon = None
        self.enforce = None

    def search(self, start_id, start_ds, length, fwd, enforce, polygon, road_network: RoadNetwork, route: Route):
        self.polygon = polygon
        self.type = fwd
        self.enforce = enforce
        if fwd:
            self._search_fwd(start_id, start_ds, length, road_network, route)
        else:
            self._search_bwd(start_id, start_ds, length, road_network)


    def _search_bwd(self, start_id, start_ds, length, road_network: RoadNetwork):
        self.start_id = start_id
        self.start_ds = start_ds
        self.length = length

        cur_lane = road_network.find_lane_id(start_id)
        lane_queue = Queue()
        lane_queue.put((cur_lane, start_ds, length))

        self.coverage_list = []

        while not lane_queue.empty():
            cur_lane, cur_end_ds, cur_remain = lane_queue.get()
            cur_lane_id = cur_lane.lane_id
            if cur_end_ds>cur_lane.length:
                for predecessor in cur_lane.predecessor:
                    new_lane = road_network.find_lane_id(predecessor)
                    new_start_ds = np.abs(cur_end_ds - new_lane.length -(new_lane.lane_id <0)*new_lane.length)
                    lane_queue.put((new_lane, new_start_ds, cur_remain))
            elif cur_end_ds<0:
                for predecessor in cur_lane.predecessor:
                    new_lane = road_network.find_lane_id(predecessor)
                    new_start_ds = np.abs(-cur_end_ds -(new_lane.lane_id <0)*new_lane.length)
                    lane_queue.put((new_lane, new_start_ds, cur_remain))
            else:
                dis_to_begin = cur_lane.delta_s_from_begin(cur_end_ds)
                if dis_to_begin < cur_remain:
                    box = cur_lane.road_bound_vertices(None, cur_end_ds)
                    self.coverage_list.append(box)
                    new_remain = cur_remain - dis_to_begin
                    for predecessor in cur_lane.predecessor:
                        new_lane = road_network.find_lane_id(predecessor)
                        new_start_ds = (new_lane.lane_id<0)*new_lane.length
                        lane_queue.put((new_lane, new_start_ds, new_remain))
                else:
                    begin_ds = cur_end_ds + np.sign(cur_lane_id)*cur_remain
                    self.end_id.append(cur_lane.lanelet_id)
                    self.end_id.append(begin_ds)
                    box = cur_lane.road_bound_vertices(begin_ds, cur_end_ds)
                    self.coverage_list.append(box)
                   




    def _search_fwd(self, start_id, start_ds, length, road_network: RoadNetwork, route: Route):
        self.start_id = start_id
        self.start_ds = start_ds
        self.length = length

        cur_lane = road_network.find_lane_id(start_id)
        lane_queue = Queue()
        lane_queue.put((cur_lane, start_ds, length))

        self.coverage_list = []

        while not lane_queue.empty():
            cur_lane, cur_start_ds, cur_remain = lane_queue.get()
            cur_lane_id = cur_lane.lane_id
            if cur_start_ds>cur_lane.length:
                for successor in cur_lane.successor:
                    new_lane = road_network.find_lane_id(successor)
                    new_start_ds = np.abs(cur_start_ds - new_lane.length -(new_lane.lane_id >0)*new_lane.length)
                    lane_queue.put((new_lane, new_start_ds, cur_remain))
            elif cur_start_ds<0:
                for successor in cur_lane.successor:
                    new_lane = road_network.find_lane_id(successor)
                    new_start_ds = np.abs(-cur_start_ds -(new_lane.lane_id >0)*new_lane.length)
                    lane_queue.put((new_lane, new_start_ds, cur_remain))
            else:
                dis_to_end = cur_lane.delta_s_from_end(cur_start_ds)
                if dis_to_end < cur_remain:
                    self.coverage_list.append(cur_lane.road_bound_vertices(cur_start_ds, None))
                    new_remain = cur_remain - dis_to_end
                    for successor in cur_lane.successor:
                        if route.in_ego_path(successor):
                            new_lane = road_network.find_lane_id(successor)
                            new_start_ds = (new_lane.lane_id>0)*new_lane.length
                            lane_queue.put((new_lane, new_start_ds, new_remain))
                else:
                    end_ds = cur_start_ds - np.sign(cur_lane_id)*cur_remain
                    self.end_id.append(cur_lane.lanelet_id)
                    self.end_id.append(end_ds)
                    self.coverage_list.append(cur_lane.road_bound_vertices(cur_start_ds, end_ds))






    
