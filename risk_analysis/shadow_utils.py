import queue
import copy
import numpy as np
from shapely.geometry.linestring import LineString
import carla
from risk_analysis.shadow import Shadow, InBound, OutBound, InOutBound, ShadowNode
from risk_analysis.visibility_checker import OctomapVisibility, DepthVisibility
from risk_analysis.reachable_set import double_integrator_reachable_tube_fb as FRS_bound
from pympler.asizeof import asizeof
import heapq
from datetime import datetime
from shapely.geometry import Polygon
from matplotlib import pyplot as plt

class ShadowUtils():
    def __init__(self, client, road_newtwork, octomap_file, route, 
                    range, ds, v_max, a_max):
        """
        ego_path: a list of (road_id, section_id, lane_id) for ego's future path
                        if we have (road_id, section_id, boolean) then it means the entire road, 
                        Ture means all positive lanes (opposite direction of the reference line)
                        False means all negative lanes (same direction of the refeence line)
        """

        self.client = client
        self.ego_id = self.client.ego.id # to be ignored during update
        self.ds = ds

        self.road_network = road_newtwork
        
        self.vis_checker_octomap = OctomapVisibility(octomap_file, self.road_network, range)
        self.vis_checker_depth = DepthVisibility(self.road_network, range)

        self.v_max = float(v_max)
        self.a_max = float(a_max)
        self.shadow_stop_length = 0.5*(self.v_max**2)/self.a_max*1.1

        self.route = route
        
        
        # keep track of vehicle that have been observed during simulation
        self.vehicle_dict = {}
        # keep track of the shadow 
        self.shadow_list = []

        self.risky_lane = []


    def add_risky_lane(self, id):
        if id not in self.risky_lane:
            self.risky_lane.append(id)
    
    def in_risky_lane(self, id):
        return id in self.risky_lane

    def predict_visibility(self, old_shadow_lists, predicted_bbox, ego_pose, predict_t):
        #dt0 = datetime.now()
        unprocessed_shadow_queue = queue.Queue()

        visibility_checker = self.vis_checker_octomap.check_visibility

        # extend the outbound by maximum distance the hidden object may travel
        predict_shadow_list = self.predict_shadow(old_shadow_lists, predict_t)
        for shadow in predict_shadow_list:
            unprocessed_shadow_queue.put(shadow)

        mature_shadow_map, mature_shadow_list = self._update_shadow(unprocessed_shadow_queue, visibility_checker, predicted_bbox, ego_pose, predict_t) 
        # process_time = datetime.now() - dt0
        # print("Takes ", process_time.total_seconds(), " sec to update")
        return mature_shadow_map, mature_shadow_list

    def update_visibility(self, sensor_data, sensor_pose, t):
        
        unprocessed_shadow_queue = queue.Queue()

        visibility_checker = self.vis_checker_depth.check_visibility

        if len(self.shadow_list) == 0:
            ego_waypoint = self.client.map.get_waypoint(self.client.ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
            ego_lane = self.road_network.find_lane(ego_waypoint.road_id, ego_waypoint.section_id, ego_waypoint.lane_id)
            ds_start = ego_waypoint.s - ego_lane.lane_section.s_start -np.sign(ego_waypoint.lane_id)*2.5
            unprocessed_shadow_queue.put(self.create_shadow(ego_lane.lanelet_id, ds_start, None, t, "in", None))
        else:
            predict_shadow_list = self.predict_shadow(self.shadow_list, t)
            for shadow in predict_shadow_list:
                unprocessed_shadow_queue.put(shadow)
            
        mature_shadow_map, mature_shadow_list = self._update_shadow(unprocessed_shadow_queue, visibility_checker, sensor_data, sensor_pose, t)
        self.shadow_list = mature_shadow_list
        
        return mature_shadow_map, mature_shadow_list

    def update_visibility_octomap(self, sensor_data, sensor_pose, t):
        
        unprocessed_shadow_queue = queue.Queue()

        visibility_checker = self.vis_checker_octomap.check_visibility

        if len(self.shadow_list) == 0:
            ego_waypoint = self.client.map.get_waypoint(self.client.ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
            ego_lane = self.road_network.find_lane(ego_waypoint.road_id, ego_waypoint.section_id, ego_waypoint.lane_id)
            ds_start = ego_waypoint.s - ego_lane.lane_section.s_start -np.sign(ego_waypoint.lane_id)*2.5
            unprocessed_shadow_queue.put(self.create_shadow(ego_lane.lanelet_id, ds_start, None, t, "in", None))
        else:
            predict_shadow_list = self.predict_shadow(self.shadow_list, t)
            for shadow in predict_shadow_list:
                unprocessed_shadow_queue.put(shadow)
            
        mature_shadow_map, mature_shadow_list = self._update_shadow(unprocessed_shadow_queue, visibility_checker, sensor_data, sensor_pose, t)
        self.shadow_list = mature_shadow_list
        
        return mature_shadow_map, mature_shadow_list
    
    def _update_shadow(self, unprocessed_shadow_queue, visibility_checker, sensor_data, sensor_pose, t):

        mature_shadow_list = []
        mature_shadow_map = {}

        def add_shadow_to_dict(shadow):
            id = shadow.root.id
            if shadow.risk_bound:
                ds = shadow.root.ds_in
            else:
                ds = shadow.root.ds_out
            if id not in mature_shadow_map:
                mature_shadow_map[id] = []
            
            heapq.heappush(mature_shadow_map[id], (ds, shadow))


        def check_repeate(shadow2check):
            id = shadow2check.root.id
            if id not in mature_shadow_map:
                return False
            else:
                shadow_list = mature_shadow_map[id]
                if shadow2check.risk_bound:
                    return self._check_repeated_fwd_shadow(shadow2check, shadow_list)
                else:
                    return self._check_repeated_bwd_shadow(shadow2check, shadow_list)
              


        # avoid repeating
        fwd_visited_lane = [] # only lanes searched from beginning to end is put in this list
        fwd_searched_begin = []

        bwd_visited_lane = [] # only lanes searched from beginning to end is put in this list
        bwd_searched_begin = []
       

        while not unprocessed_shadow_queue.empty():
            cur_shadow = unprocessed_shadow_queue.get()
            bound_lane_id = cur_shadow.root.id
                        
            # if true, update the shadow from the inbound towards out bound
            # else, update the shadow from the out bound towards in bound 
            if cur_shadow.risk_bound:

                if bound_lane_id in fwd_visited_lane:
                    continue
                                               
                if self.route.in_ego_path(cur_shadow.root.id):
                    mature_shadow, new_shadow_list, visited_lanes = self._update_fwd_shadow(cur_shadow, visibility_checker, sensor_data, sensor_pose, t)
                    fwd_visited_lane.extend(visited_lanes)
                    
                    fwd_searched_begin.append((cur_shadow.root.id, cur_shadow.root.ds_in))
                    # if a mature shadow has been formed
                    if mature_shadow is not None:
                        if not check_repeate(mature_shadow):
                            mature_shadow_list.append(mature_shadow)
                            add_shadow_to_dict(mature_shadow)
                            # if isinstance(mature_shadow.root, InOutBound):
                            #     print("find fwd io", t, mature_shadow.root.id, mature_shadow.root.ds_out, mature_shadow.root.t_out, mature_shadow.root.ds_in,  mature_shadow.root.t_in)
                            # else:
                            #     print("find fwd", t, mature_shadow.root.id, mature_shadow.root.ds_in, mature_shadow.root.t_in, mature_shadow.leaf_depth)
                                         
                    for new_shadow in new_shadow_list:
                        if new_shadow.risk_bound:
                            new_shadow_begin = (new_shadow.root.id, new_shadow.root.ds_in)
                            if new_shadow_begin not in fwd_searched_begin:
                                unprocessed_shadow_queue.put(new_shadow)
                        else:
                            new_shadow_begin = (new_shadow.root.id, new_shadow.root.ds_out)
                            if new_shadow_begin not in bwd_searched_begin:
                                unprocessed_shadow_queue.put(new_shadow)
                       
            else:
                #print("start search at", cur_shadow.root.id, cur_shadow.root.ds_out)
                if bound_lane_id in bwd_visited_lane:
                    #print("visited")
                    continue
                    
                 
                # TODO:
                # discard the backward shadow if it is no longer a risk
                if self.route.in_ego_path(cur_shadow.parent_id):
                    
                    bwd_searched_begin.append((cur_shadow.root.id, cur_shadow.root.ds_out))
                    mature_shadow, new_shadow_list, visited_lanes = self._update_bwd_shadow(cur_shadow, visibility_checker, sensor_data, sensor_pose, t)
                    #print("finish search ", visited_lanes)
                    bwd_visited_lane.extend(visited_lanes)
                    
                    
                    if mature_shadow is not None:
                        # same shadow has already be processed, discard
                        if not check_repeate(mature_shadow):
                            # if isinstance(mature_shadow.root, InOutBound):
                            #     print("find bwd io", t, mature_shadow.root.id, mature_shadow.root.ds_out, mature_shadow.root.t_out, mature_shadow.root.ds_in,  mature_shadow.root.t_in)
                            # else:
                            #     print("find bwd", t, mature_shadow.root.id, mature_shadow.root.ds_out, mature_shadow.root.t_out, mature_shadow.leaf_depth)
                            mature_shadow_list.append(mature_shadow)
                            add_shadow_to_dict(mature_shadow)
                        #else:
                            #print("repeat", mature_shadow.root.id, mature_shadow.root.ds_out)
                    # append new shadow to the queue                  
                    for new_shadow in new_shadow_list:
                        new_shadow_begin = (new_shadow.root.id, new_shadow.root.ds_out)
                        if new_shadow_begin not in bwd_searched_begin:
                            unprocessed_shadow_queue.put(new_shadow)
                            
        #print(len(mature_shadow_list))                 
        return mature_shadow_map, mature_shadow_list

    def _check_repeated_bwd_shadow(self, shadow_check: Shadow, shadow_list):
        shadow_check_leaf = []
        for leaf, depth in zip(shadow_check.leaf_list, shadow_check.leaf_depth):
            shadow_check_leaf.append((leaf.id, depth))
        
        for ds, shadow_2 in shadow_list:
            if ds == shadow_check.root.ds_out:
                found_same = True
                for leaf, depth in zip(shadow_2.leaf_list, shadow_2.leaf_depth):
                    if (leaf.id, depth) not in shadow_check_leaf:
                        found_same = False
                        break
                if found_same:
                    return True
        return False

    def _check_repeated_fwd_shadow(self, shadow_check: Shadow, shadow_list):
        shadow_check_leaf = []
        for leaf, depth in zip(shadow_check.leaf_list, shadow_check.leaf_depth):
            shadow_check_leaf.append((leaf.id, depth))
        
        for ds, shadow_2 in shadow_list:
            if ds == shadow_check.root.ds_in:
                found_same = True
                for leaf, depth in zip(shadow_2.leaf_list, shadow_2.leaf_depth):
                    if (leaf.id, depth) not in shadow_check_leaf:
                        found_same = False
                        break
                if found_same:
                    return True
        return False

    def _update_fwd_shadow(self, shadow: Shadow, visibility_checker, sensor_data, sensor_pose, t):
        """
        This search and update the shadow along a given lane's traffic flow direction
            risky_lane = (road_id, section_id, lane_id, ds_start, if_bwd=False, active_shadow, last_visible, processed_ds)
        """     
        
        new_shadow_list = []
        visited_lanes = [] #lanes that has been fully searched or where the maximum length of shadow has exceeded
        
        node_queue = queue.Queue() 
        fake_root = ShadowNode(-1)
        root_node = shadow.root
        node_queue.put((fake_root, root_node, 0))

         # initialize
        mature_shadow = shadow
        mature_shadow.leaf_list = []
        mature_shadow.leaf_depth = []
        
        
        # traverse through the shadow tree
        while not node_queue.empty():
            # processed ds is the distance to root
            parent_node, cur_node, processed_ds = node_queue.get()
            lanelet_id = cur_node.id
            lane = self.road_network.find_lane_id(lanelet_id)
            if isinstance(cur_node, InBound) or isinstance(cur_node, InOutBound):
                ds_start = cur_node.ds_in
                last_visible = True
            else:
                # if lane_id>0, then ds = [..., 10, 9, 8, ..., 3, 2, 1] in traffic direction
                ds_start = lane.length*(lane.lane_id>0) 
                last_visible = False

            idx = np.argmin(np.abs(lane.waypoint_ds-ds_start))

            if idx==0:
                start_from_begin = True
            else:
                start_from_begin = False

            waypoint_ds = lane.waypoint_ds[idx:]
            # record last waypoint
            last_ds = waypoint_ds[0] # both 0 and length are included in the waypoint ds
            continue_update = True

            for cur_ds in waypoint_ds:
                """
                check waypoint visibilty
                """
                visible = visibility_checker(lane, cur_ds, sensor_data, sensor_pose, t)

                """
                Update the shadow
                """
                # update visibility
                lane.update(cur_ds, (t, visible)) 
                processed_ds += np.abs(last_ds - cur_ds)

                if self._check_fully_processed(cur_ds, cur_node, "fwd"):
                    if not visible or not last_visible:
                        cur_node.child_node = None
                        self._append_child_node(parent_node, cur_node) 
                        mature_shadow.leaf_list.append(cur_node)
                        mature_shadow.leaf_depth.append(processed_ds)
                    continue_update = False
                    break

                if not visible:                                                    
                    if processed_ds >= self.shadow_stop_length:
                        # this branch is terminated since it reaches the threshold
                        #if not isinstance(cur_node, InOutBound): #No way
                        if isinstance(cur_node, OutBound):
                            cur_node = ShadowNode(lanelet_id)
                        else:
                            cur_node.child_node = None
                        self._append_child_node(parent_node, cur_node) 
                        if start_from_begin:
                            visited_lanes.append(lanelet_id) # further check in this lane is unneccessary
                        mature_shadow.leaf_list.append(cur_node)
                        mature_shadow.leaf_depth.append(None)
                        continue_update = False
                        break
                else:
                    if last_visible:
                        # this only occurs if the current node is an inbound. We update the inbound fwd.
                        # if all waypoints of the lane containing an inbound are visible, we discard the current shadow, 
                        # and create new shadows whose inbounds are at the beginning of each child nodes.

                        # otherwise, shrink the InBound
                        if processed_ds>0:
                            
                            if cur_node.ds_in_hist is not None:
                                new_ds_in_hist = []
                                new_t_in_hist = []
                                for ds_in_hist, t_in_hist in zip(cur_node.ds_in_hist, cur_node.t_in_hist):
                                    ds_in_hist += processed_ds
                                    if cur_node.t_in < t:
                                        new_ds_in_hist.append(np.hstack((ds_in_hist, processed_ds)))
                                        new_t_in_hist.append(np.hstack((t_in_hist, cur_node.t_in)))
                                    else:
                                        new_ds_in_hist.append(ds_in_hist)
                                        new_t_in_hist.append(t_in_hist)
                                cur_node.ds_in_hist = new_ds_in_hist
                                cur_node.t_in_hist = new_t_in_hist
                            # initialize the inbound histogram 
                            elif cur_node.t_in < t:
                                cur_node.ds_in_hist = [np.array([processed_ds])]
                                cur_node.t_in_hist = [np.array([cur_node.t_in])]
                        
                            # update node information
                            cur_node.ds_in = cur_ds
                            cur_node.t_in = t
                            processed_ds = 0 # w.r.t the most recent inbound
                    else:
                        
                        # create a new shadow tree whose in bound is the current waypoint that will be updated fwd later
                        new_shadow_list.append(self._split_fwd_shadow(cur_node, cur_ds, None, t, root_node, processed_ds))
                        
                        if isinstance(cur_node, InBound) or isinstance(cur_node, InOutBound):
                            # make the current waypoint as an InOutBound
                            new_node = InOutBound(lanelet_id, cur_node.ds_in, cur_ds, cur_node.t_in, t)
                            new_node.ds_in_hist = copy.deepcopy(cur_node.ds_in_hist)
                            new_node.t_in_hist = copy.deepcopy(cur_node.t_in_hist)
                        else:
                            # make the current waypoint as an out bound
                            new_node = OutBound(lanelet_id, cur_ds, t)
                        
                        self._append_child_node(parent_node, new_node)
                        mature_shadow.leaf_list.append(new_node)
                        mature_shadow.leaf_depth.append(processed_ds)

                        continue_update = False
                        break
                        
                last_ds = cur_ds
                last_visible = visible
            
            if continue_update:  
                if start_from_begin:
                    visited_lanes.append(lanelet_id)
                
                if last_visible:
                    if cur_node.child_node is None:
                        # search in the road network
                        for successor in lane.successor:
                            # create a new shadow tree whose in bound is the current waypoint that will be updated fwd later
                            new_shadow = self.create_shadow(successor, None, 0, t, "in", None)
                            # carry over the old observations
                            # Simply copy it because the waypoint includes both start and end point.
                            # so that the end of current lane is the start of the successor lane
                            new_shadow.root.ds_in_hist =  copy.deepcopy(root_node.ds_in_hist)
                            new_shadow.root.t_in_hist = copy.deepcopy(root_node.t_in_hist)
                            new_shadow_list.append(new_shadow) 
                    else:
                        for child in cur_node.child_node:
                            # create a new shadow tree whose in bound is the current waypoint that will be updated fwd later
                            new_shadow_list.append(self._split_fwd_shadow(child, None, 0, t, root_node, 0))
                else:
                    #if isinstance(cur_node, ShadowNode):
                    
                    # keep expanding the tree
                    if cur_node.child_node is None:
                        # search in the road network
                        for successor in lane.successor:
                            node_queue.put((cur_node, ShadowNode(successor), processed_ds))
                    else:
                        for child in cur_node.child_node:
                            node_queue.put((cur_node, child, processed_ds))
                    cur_node.child_node = None
                    self._append_child_node(parent_node, cur_node) # append the node back to the tree
                
            # put the adjcant, merge and cross lanes in the new shadow for check
            for neighbor in lane.adj_same:
                new_shadow_list.append(self.create_shadow(neighbor, ds_start, None, t, "in", None))

            for neighbor in lane.adj_oppo:
                new_shadow_list.append(self.create_shadow(neighbor, ds_start, None, t, "out", lanelet_id))
                self.add_risky_lane(neighbor)
            
            if lane.merge is not None:
                for neighbor in lane.merge:
                    new_shadow_list.append(self.create_shadow(neighbor, None, 0, t, "out", lanelet_id))
                    self.add_risky_lane(neighbor)
            
            if lane.cross is not None:
                for neighbor in lane.cross:
                    new_shadow_list.append(self.create_shadow(neighbor, None, 0, t, "out", lanelet_id))
                    self.add_risky_lane(neighbor)
            
        if fake_root.child_node is None:
            mature_shadow = None
        else:
            mature_shadow.root = fake_root.child_node[0]
            mature_shadow.root.parent_node = None
            
            #print(t, mature_shadow.root.id, mature_shadow.root.ds_in, mature_shadow.root.t_in)

        return mature_shadow, new_shadow_list, visited_lanes
            
    def _update_bwd_shadow(self, shadow: Shadow, visibility_checker, sensor_data, sensor_pose, t):
        """
        This search and update the shadow along a given lane's traffic flow direction
            risky_lane = (road_id, section_id, lane_id, ds_start, if_bwd=False, active_shadow, last_visible, processed_ds)
        """     

        new_shadow_list = []
        visited_lanes = [] #lanes that has been fully searched or where the maximum length of shadow has exceeded
        mature_shadow = shadow

        node_queue = queue.Queue() 
        fake_root = ShadowNode(-1)
        root_node = shadow.root
        node_queue.put((fake_root, root_node, 0))

        # initialize
        mature_shadow.leaf_list = []
        mature_shadow.leaf_depth = []

        #print("bwd search begin at ", shadow.root.id, shadow.root.ds_out)
       
        # traverse through the shadow tree
        while not node_queue.empty():
            # processed ds is the distance to root
            parent_node, cur_node, processed_ds = node_queue.get()
            lanelet_id = cur_node.id
            lane = self.road_network.find_lane_id(lanelet_id)
            if isinstance(cur_node, OutBound) or isinstance(cur_node, InOutBound):
                ds_start = cur_node.ds_out
                last_visible = True
            else:
                # if lane_id >0 , then ds = [..., 10, 9, 8, ..., 3, 2, 1] in traffic direction
                ds_start = lane.length*(lane.lane_id<0) 
                last_visible = False

            idx = np.argmin(np.abs(lane.waypoint_ds-ds_start))       
               
            waypoint_ds = np.flip(lane.waypoint_ds[:idx+1]) 
            
            if idx==(lane.waypoint_ds.shape[0]-1):
                start_from_end = True
            else:
                start_from_end = False
       
            # record last waypoint
            last_ds = waypoint_ds[0] # both 0 and length are included in the waypoint ds
            continue_update = True

            for cur_ds in waypoint_ds:
                """
                check waypoint visibilty
                """
                visible = visibility_checker(lane, cur_ds, sensor_data, sensor_pose, t)

                """
                Update the shadow
                """
                # update visibility
                lane.update(cur_ds, (t, visible)) 
                processed_ds += np.abs(last_ds - cur_ds)

                if self._check_fully_processed(cur_ds, cur_node, "bwd"):
                    if not visible or not last_visible:
                        cur_node.child_node = None
                        self._append_child_node(parent_node, cur_node) 
                        mature_shadow.leaf_list.append(cur_node)
                        mature_shadow.leaf_depth.append(processed_ds)
                    continue_update = False
                    #print("reach in bound at ", lanelet_id, cur_ds, processed_ds)
                    break

                if not visible:                                                    
                    if processed_ds >= self.shadow_stop_length:
                        # this branch is terminated since it reaches the threshold
                        if isinstance(cur_node, InBound):
                            cur_node = ShadowNode(lanelet_id)
                        else:
                            cur_node.child_node = None
                        self._append_child_node(parent_node, cur_node) 
                        if start_from_end:
                            visited_lanes.append(lanelet_id) # further check in this lane is unneccessary
                        mature_shadow.leaf_list.append(cur_node)
                        mature_shadow.leaf_depth.append(None)
                        continue_update = False
                        break
                else:
                    if last_visible:
                        # shrink the OutBound                        
                        cur_node.id = lanelet_id
                        cur_node.ds_out = cur_ds
                        cur_node.t_out = t
                        processed_ds = 0 # w.r.t the most recent inbound
                    else:
                        # New in bound discovered
                        # otherwise create a new shadow whose out bound is the current waypoint that will be updated fwd later
                        new_shadow_list.append(self._split_bwd_shadow(cur_node, cur_ds, None, t,  shadow.parent_id))
                        
                        # Make the current waypoint as a new leaf for the current shadow
                        if isinstance(cur_node, OutBound) or isinstance(cur_node, InOutBound):
                            # cur_node is an OutBound, which means it is the root
                            new_node = InOutBound(lanelet_id, cur_ds, cur_node.ds_out, t, cur_node.t_out)
                            length = np.abs(cur_node.ds_out - cur_ds)
                        else:
                            # make the waypoint as an in bound
                            new_node = InBound(lanelet_id, cur_ds, t)
                            length = self._partial_lane_length(lane, cur_ds, False)
                        
                        # recursively retrive InBound History from 
                        new_ds_in_hist, new_t_in_hist = self._traverse_bwd_shadow(cur_node, -length)
                        #a list of (list of np.array, list of np.array, processed length)
                        if len(new_ds_in_hist)>0:
                            new_node.ds_in_hist = new_ds_in_hist
                            new_node.t_in_hist = new_t_in_hist
                                                
                        self._append_child_node(parent_node, new_node)
                        mature_shadow.leaf_list.append(new_node)
                        mature_shadow.leaf_depth.append(processed_ds)
                        
                        continue_update = False
                        break
                        
                last_ds = cur_ds
                last_visible = visible
                 
            if continue_update:
                if start_from_end:
                    visited_lanes.append(lanelet_id)
                
                if last_visible:
                    if cur_node.child_node is None:
                        # search in the road network
                        for predecessor in lane.predecessor:
                            # create a new shadow tree whose out bound is the current waypoint that will be updated bwd later
                            new_shadow_list.append(self.create_shadow(predecessor, None, 0, t, "out", shadow.parent_id))
                            self.add_risky_lane(predecessor)
                    else:
                        for child in cur_node.child_node:
                            new_shadow_list.append(self._split_bwd_shadow(child, None, 0, t, shadow.parent_id))                 
                else:
                    #if isinstance(cur_node, ShadowNode):
                    
                    # keep expanding the tree
                    
                    if cur_node.child_node is None:
                        # search in the road network
                        for predecessor in lane.predecessor:
                            node_queue.put((cur_node, ShadowNode(predecessor), processed_ds))
                            self.add_risky_lane(predecessor)
                    else:
                        for child in cur_node.child_node:
                            node_queue.put((cur_node, child, processed_ds))
                    cur_node.child_node = None
                    self._append_child_node(parent_node, cur_node) # append the node back to the tree
                   
                    
        if fake_root.child_node is None:
            mature_shadow = None
        else:
            # if mature_shadow.root.id == 206:
            #     print("BWD", t, mature_shadow.root.id, mature_shadow.root.ds_out, mature_shadow.root.t_out)
            mature_shadow.root = fake_root.child_node[0]
            mature_shadow.root.parent_node = None
        return mature_shadow, new_shadow_list, visited_lanes

    def _split_bwd_shadow(self, node_split, ds, ds_rel, t, parent_id):
        """
        split a bwd shadow tree by creating new root (outbound) at (node_split, ds)
        that will be updated bwd later
        """
        new_shadow = self.create_shadow(node_split.id, ds, ds_rel, t, "out", parent_id)
        # carry over the old observations
        if isinstance(node_split, InBound) or isinstance(node_split, InOutBound):
            # current node is an out bound, carry it over to the new shadow
            new_node = InOutBound(node_split.id, node_split.ds_in, new_shadow.root.ds_out, node_split.t_in, new_shadow.root.t_out)
            # do not worry about leaf list, they will be updated when checking visibility
            new_node.t_in_hist = copy.deepcopy(node_split.t_in_hist)
            new_node.ds_in_hist = copy.deepcopy(node_split.ds_in_hist)
            new_shadow.root = new_node
        else:
            # inherient the child nodes of current node
            new_shadow.root.child_node = node_split.child_node
        return new_shadow

    def _split_fwd_shadow(self, node_split, ds, ds_rel, t, original_root, processed_ds):
        """
        split a fwd shadow tree by creating new root (outbound) at (node_split, ds)
        carry over inbound history from original_root and updates them later
        """
        new_shadow = self.create_shadow(node_split.id, ds, ds_rel, t, "in", None)
        if isinstance(node_split, OutBound) or isinstance(node_split, InOutBound):
            # make the new_shadow root as an InOutBound
            new_node = InOutBound(node_split.id, new_shadow.root.ds_in, node_split.ds_out, new_shadow.root.t_in, node_split.t_out)
            new_shadow.root = new_node
        else:
            new_shadow.root.child_node = node_split.child_node

        if processed_ds > 0:
            if original_root.ds_in_hist is not None:
                new_ds_in_hist = []
                new_t_in_hist = []
                for ds_in_hist, t_in_hist in zip(original_root.ds_in_hist, original_root.t_in_hist):
                    ds_in_hist += processed_ds
                    if original_root.t_in < t:
                        new_ds_in_hist.append(np.hstack((ds_in_hist, processed_ds)))
                        new_t_in_hist.append(np.hstack((t_in_hist, original_root.t_in)))
                    else:
                        new_ds_in_hist.append(ds_in_hist)
                        new_t_in_hist.append(t_in_hist)
                new_shadow.root.ds_in_hist = new_ds_in_hist
                new_shadow.root.t_in_hist = new_t_in_hist
            # initialize the inbound histogram 
            elif original_root.t_in < t:
                new_shadow.root.ds_in_hist = [np.array([processed_ds])]
                new_shadow.root.t_in_hist = [np.array([original_root.t_in])]
        else:
            new_shadow.root.ds_in_hist =  copy.deepcopy(original_root.ds_in_hist)
            new_shadow.root.t_in_hist = copy.deepcopy(original_root.t_in_hist)
        return new_shadow

    def _check_fully_processed(self, ds, node2check, type):
        """
            type : "fwd" or "bwd"
        """

        if type == "fwd":
            if isinstance(node2check, OutBound) or isinstance(node2check, InOutBound):
                lane = self.road_network.find_lane_id(node2check.id)
                #this branch is terminated since the all waypoints covered by the shadow in the previous obversation and its reachable sets have been explored
                if (lane.lane_id>0 and ds<=node2check.ds_out) or (lane.lane_id<0 and ds>=node2check.ds_out):
                    return True
        else:
            if isinstance(node2check, InBound) or isinstance(node2check, InOutBound):
                lane = self.road_network.find_lane_id(node2check.id)
                #this branch is terminated since the all waypoints covered by the shadow in the previous obversation and its reachable sets have been explored
                if (lane.lane_id>0 and ds>=node2check.ds_in) or (lane.lane_id<0 and ds<=node2check.ds_in):
                    return True
        return False

    def _traverse_bwd_shadow(self, cur_node, processed_length):
        lane = self.road_network.find_lane_id(cur_node.id)
        new_ds_in_hist = []
        new_t_in_hist = []
        
        if isinstance(cur_node, InBound) or isinstance(cur_node, InOutBound):
            # base case
            if isinstance(cur_node, InBound):
                shadow_length  = processed_length + self._partial_lane_length(lane, cur_node.ds_in, False)
            else:
                shadow_length = np.abs(cur_node.ds_out - cur_node.ds_in) + processed_length
            if cur_node.ds_in_hist is None:
                new_ds_in_hist = [np.array([shadow_length])]
                new_t_in_hist = [np.array([cur_node.t_in])]
            else:
                for ds_in_hist, t_in_hist in zip(cur_node.ds_in_hist, cur_node.t_in_hist):
                    ds_in_hist += shadow_length
                    ds_in_hist  = np.hstack((ds_in_hist, shadow_length))
                    t_in_hist = np.hstack((t_in_hist, cur_node.t_in))
                    new_ds_in_hist.append(ds_in_hist)
                    new_t_in_hist.append(t_in_hist)
            return new_ds_in_hist, new_t_in_hist
        elif cur_node.child_node is not None: 

            if isinstance(cur_node, OutBound):
                new_processed_length = processed_length+self._partial_lane_length(lane, cur_node.ds_out, True)
            else:
                new_processed_length = processed_length+lane.length
            for child in cur_node.child_node:
                ds_in_hist, t_hist = self._traverse_bwd_shadow(child, new_processed_length)
                new_ds_in_hist.extend(ds_in_hist)
                new_t_in_hist.extend(t_hist)
        
        return new_ds_in_hist, new_t_in_hist   
        
    def create_shadow(self, id, ds, ds_rel, t, type: str, parent_id):
        lane = self.road_network.find_lane_id(id)
        shadow = Shadow()
        if type=="in":
            if ds is None:
                if lane.lane_id < 0:
                    ds = ds_rel
                else:
                    ds = lane.length - ds_rel
            shadow.risk_bound = True
            shadow.root = InBound(id, ds, t)

        elif type=="out":
            if ds is None:
                if lane.lane_id < 0:
                    ds = lane.length - ds_rel
                else:
                    ds = ds_rel
            shadow.risk_bound = False
            shadow.root = OutBound(id, ds, t)
            shadow.parent_id = parent_id
        return shadow

    def _remove_old_inbound(self, in_bound, t):
        if in_bound.ds_in_hist is None:
            return
        
        step_mature = (t - self.v_max/self.a_max) # an obstacle can accelerate to the maximum speed from static 
        new_ds_in_hist = []
        new_t_in_hist = []
        for ds_in_hist, t_in_hist in zip(in_bound.ds_in_hist, in_bound.t_in_hist):
            idx_mature = np.searchsorted(t_in_hist, step_mature, side = 'right')
            if idx_mature > 1:
                new_ds_in_hist.append(ds_in_hist[idx_mature-1:])
                new_t_in_hist.append(t_in_hist[idx_mature-1:])
                #print("remove old inbound",self.v_max/self.a_max, step_mature, idx_mature, t_in_hist)
            else:
                new_ds_in_hist.append(ds_in_hist)
                new_t_in_hist.append(t_in_hist)

        in_bound.ds_in_hist = new_ds_in_hist
        in_bound.t_in_hist = new_t_in_hist
    
    def calc_V_max(self, in_bound, dis, t):
        V_max_list = []
        if in_bound.ds_in_hist is None:
            ds_in_hist_list = [np.array([0])]
            t_in_hist_list = [np.array([in_bound.t_in])]
        else:
            ds_in_hist_list = [np.hstack((ds, 0)) for ds in in_bound.ds_in_hist]
            t_in_hist_list = [np.hstack((t_in_hist, in_bound.t_in)) for t_in_hist in in_bound.t_in_hist]

        for ds_in_hist, t_in_hist in zip(ds_in_hist_list, t_in_hist_list):
            delta_s = ds_in_hist+dis# distance between outbound and inbound
            delta_t = (t - t_in_hist)            
            V_in_max = (delta_s - 0.5*self.a_max*delta_t**2)/delta_t
            V_in_max[delta_t == 0] = self.v_max
            V_in_max[V_in_max<0] = 0
            V_out_max = np.sqrt(V_in_max**2+2*self.a_max*delta_s)
            
                
            V_out_max[V_out_max>self.v_max] = self.v_max
            V_max_list.append(V_out_max.min())

        return max(V_max_list)
        
    def _calc_S_max(self, v0, dt):
        #print(v0, dt)
        if v0>= self.v_max:
            return dt*v0, self.v_max
        elif v0==0:
            return 0, 0
        
        t_to_max = (self.v_max-v0)/self.a_max

        if dt < t_to_max:
            return v0*dt+0.5*self.a_max*dt**2, v0+self.a_max*dt
        else:
            return v0*dt+0.5*self.a_max*t_to_max**2+(dt-t_to_max)*self.v_max, self.v_max

    @staticmethod
    def _append_child_node(parent_node, cur_node):
        if parent_node.child_node is None:
            parent_node.child_node = [cur_node]
        else:
            parent_node.child_node.append(cur_node)
        cur_node.parent_node = parent_node

    @staticmethod
    def _partial_lane_length(lane, ds, from_start:bool, round = False):
        """
        Given a lane, calculate the length from start/end of this lane section to the ds
        """
        # calculate from start of the lane (based on traffic direction)
        if from_start: 
            length = np.abs((lane.lane_id>0)*lane.length - ds)
        else:
            length = np.abs((lane.lane_id<0)*lane.length - ds)
        
        if round:
            idx = np.argmin(np.abs(lane.waypoint_ds - length))
            return lane.waypoint_ds[idx]
        else:
            return length
        
    def get_shadow_for_plot(self, shadow_to_plot = None):
        if shadow_to_plot is None:
            shadow_to_plot = self.shadow_list
        shadow_vertice_list = []
        for shadow in shadow_to_plot:
            node_queue = queue.Queue()
            node_queue.put(shadow.root)
                        
            while not node_queue.empty():
                cur_node = node_queue.get()
                lane = self.road_network.find_lane_id(cur_node.id)
                if isinstance(cur_node, InOutBound):
                    shadow_vertice_list.append(lane.road_bound_vertices(cur_node.ds_in, cur_node.ds_out))
                elif isinstance(cur_node, InBound):
                    shadow_vertice_list.append(lane.road_bound_vertices(cur_node.ds_in, None))
                elif isinstance(cur_node, OutBound):
                    shadow_vertice_list.append(lane.road_bound_vertices(None, cur_node.ds_out))
                else:
                    shadow_vertice_list.append(lane.road_bound_vertices(None, None))
                
                if not isinstance(cur_node, InOutBound) and cur_node.child_node is not None:
                    for child in cur_node.child_node:
                        node_queue.put(child)

        return shadow_vertice_list

    # TODO: Although never occured in our test case
    # what if it extend to more than one lane?
    # what if it extend to the lane that is not interested 

    def extent_outbound(self, outbound, v_max, t, bwd_shadow = False):
        """
        Given the max velocity that the object may exit the shadow,
        find the furthest point the object may reach and its max possible velocity
        """

        lane = self.road_network.find_lane_id(outbound.id)
        s_extend, v_FRS = self._calc_S_max(v_max, (t-outbound.t_out))
        if s_extend == 0:
            # Keep the original outbound
            return (outbound.id, outbound.ds_out)

        def _extent(ds_extent):
            extent_outbound_list = []
            for successor in lane.successor:
                successor_lane = self.road_network.find_lane_id(successor)
                ds_successor = self._partial_lane_length(successor_lane, ds_extent, True, True)
                extent_outbound_list.append((successor, ds_successor))
            return extent_outbound_list
        
        def _extent_bwd_shadow(ds_extent, ds_end):
            extent_outbound_list = []
            for successor in lane.successor:
                if self.in_risky_lane(successor):
                    successor_lane = self.road_network.find_lane_id(successor)
                    ds_successor = self._partial_lane_length(successor_lane, ds_extent, True, True)
                    extent_outbound_list.append((successor, ds_successor))
            if len(extent_outbound_list)==0:
                return (outbound.id, ds_end)
            else:
                return extent_outbound_list

        ds_FRS = outbound.ds_out - np.sign(lane.lane_id)*s_extend
        #("extent from ", outbound.id, outbound.ds_out, "by ", s_extend)
        if 0<=ds_FRS<=lane.length:
            return (outbound.id, ds_FRS)
        elif ds_FRS < 0:
            ds_extent = -ds_FRS
            if bwd_shadow:
                return _extent_bwd_shadow(ds_extent, 0)
            else:
                return _extent(-ds_FRS)
            
        else:
            ds_extent = ds_FRS - lane.length
            if bwd_shadow:
                return _extent_bwd_shadow(ds_extent, lane.length)
            else:
                return _extent(ds_FRS - lane.length)
            
    def predict_fwd_shadow(self, shadow: Shadow, t):
        shadow_copy = copy.deepcopy(shadow)
        self._remove_old_inbound(shadow_copy.root, t)
        
        for outbound, depth in zip(shadow_copy.leaf_list, shadow_copy.leaf_depth):
            if depth is None:
                continue
            elif depth < 3:
                v_max = 0 # too short for vehicle
            else:
                v_max = self.calc_V_max(shadow_copy.root, depth-3, t)
            extent_outbound_list = self.extent_outbound(outbound, v_max, t)
            if isinstance(extent_outbound_list, tuple):
                # extent out is in the original lane
                outbound.ds_out = extent_outbound_list[1]
                outbound.t_out = t
            else: 
                # outbound extend to the successor of current lane
                if isinstance(outbound, InOutBound):
                    new_cur_node = InBound(outbound.id, outbound.ds_in, outbound.t_in)
                    new_cur_node.ds_in_hist = copy.deepcopy(outbound.ds_in_hist)
                    new_cur_node.t_in_hist = copy.deepcopy(outbound.t_in_hist)
                    shadow.root = new_cur_node
                else:
                    new_cur_node = ShadowNode(outbound.id)
                    outbound.parent_node.child_node.remove(outbound)
                    outbound.parent_node.child_node.append(new_cur_node)
                new_cur_node.child_node = []
                for id, ds_out in extent_outbound_list:
                    new_outbound = OutBound(id, ds_out, t)
                    new_cur_node.child_node.append(new_outbound)
                            

        return shadow_copy

    def predict_bwd_shadow(self, shadow: Shadow, t):
        
        v_max_list = []
        for inbound, depth in zip(shadow.leaf_list, shadow.leaf_depth):
            if depth is None:
                v_max_list.append(self.v_max)
                break
            if depth < 3:
                v_max_list.append(0)
            else:
                v_max_list.append(self.calc_V_max(inbound, depth-3, t))   
        v_max = max(v_max_list)
       
        extent_outbound_list = self.extent_outbound(shadow.root, v_max, t,  True)
        if isinstance(extent_outbound_list, tuple):
            # extent out is in the original lane
            shadow_copy = copy.deepcopy(shadow)
            shadow_copy.root.ds_out = extent_outbound_list[1]
            shadow_copy.root.t_out = t
            return [shadow_copy]
        else:
            shadow_copy_list = []
            for id, ds_out in extent_outbound_list:
                shadow_copy = copy.deepcopy(shadow)
                new_out_bound = OutBound(id, ds_out, t)
                
                if isinstance(shadow_copy.root, OutBound):
                    # first make the original root to a ShadowNode 
                    new_node = ShadowNode(shadow_copy.root.id)
                    new_node.child_node = copy.deepcopy(shadow_copy.root.child_node)
                    new_out_bound.child_node = [new_node]
                elif isinstance(shadow_copy.root, InOutBound):
                    # first make the original root to a InBound
                    new_node = InBound(shadow_copy.root.id, shadow_copy.root.ds_in, shadow_copy.root.t_in)
                    new_node.ds_in_hist = shadow_copy.root.ds_in_hist
                    new_node.t_in_hist = shadow_copy.root.t_in_hist
                    new_out_bound.child_node = [new_node]
                    shadow_copy.root = new_out_bound
                shadow_copy_list.append(shadow_copy)
            return shadow_copy_list

    def predict_shadow(self, shadow_list, t):
        """ duplicate and extent the outbound for all shadows observed from last time step"""
        new_shadow_list = []
        for shadow in shadow_list:
            if shadow.risk_bound:
                new_shadow_list.append(self.predict_fwd_shadow(shadow, t))
            else:
                new_shadow_list.extend(self.predict_bwd_shadow(shadow, t))
        return new_shadow_list

    def bwd_shadow_FRS(self, shadow: Shadow, t_sense, t_predict, n=20, oppo = False):
        # TODO
        # calculate this in a matrix form instead of nesty for loop
        delta_t = t_predict - t_sense
        # if t_sense != shadow.root.t_out:
        #     raise ValueError(t_sense, shadow.root.t_out)

        FRS_union = None
        static_obj = None
        static_length = 0
        for inbound, depth in zip(shadow.leaf_list, shadow.leaf_depth):
            if depth is None:
                v_FRS, dis_FRS = FRS_bound(0, self.v_max, delta_t, delta_t, self.a_max, 0, self.v_max)
                shell = []
                for dis, v in zip(dis_FRS, v_FRS):
                    shell.append((dis, v))
                shell.append((-200, self.v_max))
                shell.append((-200, 0))
            elif depth > 3:
                v_max_cur = self.calc_V_max(inbound, depth-3, t_sense)
                #print("last obsered at", t_sense, "with", v_max_cur)
                max_extent,_ = self._calc_S_max(v_max_cur, delta_t)
                v_FRS, dis_FRS = FRS_bound(0, v_max_cur, delta_t, delta_t, self.a_max, 0, self.v_max)

                dis_list = np.linspace(0, depth-3+max_extent, n)
                v_max_list = [self.calc_V_max(inbound, dis, t_predict) for dis in dis_list]
                #print(depth, v_max_list, inbound.t_in, inbound.t_in_hist, t_predict)
                if v_max_list[0]>0:
                    dis_list = np.hstack((0, dis_list))
                    v_max_list.insert(0, 0)
                
                # plt.plot(dis_list - depth, v_max_list, '-')
                # plt.plot(dis_FRS, v_FRS, '--')
                # plt.show()
                shell = []

                for dis, v in zip(dis_list, v_max_list):
                    shell.append((dis - depth, v))
                
                for dis, v in zip(np.flip(dis_FRS), np.flip(v_FRS)):
                    shell.append((dis, v))
            else:
                if depth> static_length:
                    static_obj = LineString([(0,0), (-depth, 0)])
                    static_length = depth
                continue
            if oppo:
                shell_oppo = []
                for s,v in shell:
                    shell_oppo.append((-s,-v))
                shell = shell_oppo
            if FRS_union is None:
                FRS_union = Polygon(shell)
            else:
                FRS_union = FRS_union.union(Polygon(shell))

        if FRS_union is None:
            FRS_union = static_obj
        return FRS_union
            

                    
               