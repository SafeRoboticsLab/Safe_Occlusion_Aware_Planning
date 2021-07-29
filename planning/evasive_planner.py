from os import sendfile
from client import SynchronousClient
from mapping import RoadNetwork
from planning import Route
from risk_analysis.reachable_set import double_integrator_stop_dangerzone as stop_dangerzone
from risk_analysis.reachable_set import translate_polygon
from risk_analysis.reachable_set import double_integrator_reachable_tube as reachable_tube
import numpy as np
import heapq

import matplotlib.pyplot as plt


class Evasion():
    def __init__(self, client:SynchronousClient, road_network: RoadNetwork, route: Route, v_max, a_max):
        
        self.client = client
        self.road_network = road_network
        self.route = route
        self.v_max = v_max*1.2 # allow other agent to be 20% over speed limit
        self.a_max = a_max
        self.ego_width = 2
        self.ego_length = 5
        self.slack_distance = 4
        self.debug = False

    def plan_close_loop(self, p_ego, l_ego, v_ego, 
                        t_cur, t_plan, obstacle_map, shadow_map):

        """
        Check if the ego object can safely planned to (nominal_lanelet_id, ds, dl) at t_plan based on observation at t_cur
        """

        # first determine the state
        nominal_lanelet_id, ds = self.route.find_nominal_lane(p_ego)
        lanelet_id, direction = self.route.find_lanelet_id(nominal_lanelet_id, l_ego, self.ego_width)
               
        if len(lanelet_id) == 1 and direction[0]:
            # occuping only one ego lane
            return self._close_loop_ego_lane(lanelet_id[0], ds, p_ego, l_ego, v_ego, 
                                                        t_cur, t_plan, obstacle_map, shadow_map)
            
        else:
            raise RuntimeError

    def _ego_lane_explore(self, lanelet_id, ds, search_stop, obstacle_map, shadow_map):
        
        """
        From (lanelet_id, ds) search forward, find the closest obstacle, shadow, intersection and lane end ahead within the stop distance
        at the time of sense
        """
        if self.debug:
            print("search stop at", search_stop)
        cur_lane = self.road_network.find_lane_id(lanelet_id)
        cur_lanelet_id = cur_lane.lanelet_id
        p_covered = -cur_lane.delta_s_from_begin(ds) 

        first_obstacle_ahead = None
        
        first_shadow = None
        first_intersection = None
        lane_end = None

        p_rel_obs = np.inf

        # get an idea of what is ahead
        while p_covered < search_stop:
            if self.debug:
                print("current covered", p_covered, "search ", cur_lanelet_id)
            if first_obstacle_ahead is None:
                if cur_lanelet_id in obstacle_map:
                    cur_obstacle_queue = obstacle_map[cur_lanelet_id]
                    if cur_lane.lane_id<0: 
                        # the closest object have smaller ds
                        sorted_obstacle = heapq.nsmallest(len(cur_obstacle_queue), cur_obstacle_queue)
                    else: 
                        # the closest object have larger ds
                        sorted_obstacle = heapq.nlargest(len(cur_obstacle_queue), cur_obstacle_queue)
                    for obs_ds, v_frenet, obstacle in sorted_obstacle:
                        p_rel = cur_lane.delta_s_from_begin(obs_ds)+p_covered

                        if p_rel < 0:
                            continue
                        else:
                            first_obstacle_ahead = (obstacle, v_frenet, p_rel)
                            p_rel_obs = p_rel
                            break

            if first_shadow is None:
                # Find the first shadow ahead. If it is intersected with danger zone, retun 2, distance to shadow
                if cur_lanelet_id in shadow_map:
                    cur_shadow_queue = shadow_map[cur_lanelet_id]
                    if cur_lane.lane_id<0: 
                        # the closest object have smaller ds
                        sorted_shadow = heapq.nsmallest(len(cur_shadow_queue), cur_shadow_queue)
                    else: 
                        # the closest object have larger ds
                        sorted_shadow = heapq.nlargest(len(cur_shadow_queue), cur_shadow_queue)
                    for shadow_ds, shadow in sorted_shadow:
                        # TODO how about shadow merge into the current lane
                        p_rel = cur_lane.delta_s_from_begin(shadow_ds)+p_covered
                        # consider the worst case for the shadow is some static obstacle
                        if p_rel <= 0:
                            # this only occur at the lane that ego vehicle occupied
                            continue
                        elif p_rel<=p_rel_obs:
                            first_shadow = (shadow, p_rel)
                            break       
                       
            if first_intersection is None:
                # check if current lane is an intersection, and the car can break before intersection
                if cur_lane.cross is not None:
                    first_intersection = (cur_lanelet_id, p_covered)

            p_covered += cur_lane.length

            if p_covered < search_stop:
                find_successor = False
                for successor in cur_lane.successor:
                    if self.route.in_ego_path(successor):
                        find_successor = True
                        cur_lanelet_id = successor
                        cur_lane = self.road_network.find_lane_id(cur_lanelet_id)
                        break
                if not find_successor:
                    lane_end = (cur_lanelet_id, p_covered)
                    break

        return first_obstacle_ahead, first_shadow, first_intersection, lane_end

    def _close_loop_ego_lane(self, lanelet_id_sense, ds_sense, p_sense, l_ego, v_sense, t_sense, t_plan, obstacle_map, shadow_map):
        """ check if the ego stay in the same lane"""

        delta_t = t_plan-t_sense
        danger_zone, stop_distance = stop_dangerzone(v_sense, self.a_max, self.v_max, x_extend=self.ego_length/2)
        if self.debug:
            print("in _is_safe_plan_ego_lane", stop_distance)
        # given the pose of sense, 

        first_obstacle_ahead, first_shadow, first_intersection, lane_end = self._ego_lane_explore(lanelet_id_sense, ds_sense, 
                                                        1.5*self.v_max**2/(2*self.a_max), obstacle_map, shadow_map)
        safe = True
        if self.debug:
            print(first_intersection)

        a_brake_shadow = self.a_max
        a_brake_obstacle = self.a_max
        a_brake_lane_end = self.a_max

        v_check = self.v_max

        if first_shadow is not None:
            _, p_rel = first_shadow
            if p_rel < (stop_distance+self.slack_distance):
                # impossible to brake before shadow, consider lanechange
                raise RuntimeError("Unsafe due to shadow")
            else:
                a_brake_shadow = min(a_brake_shadow, self._max_accel(v_sense, 0, 0, p_rel, delta_t))
                v_check = 0
        
        if lane_end is not None:
            _, p_end = lane_end
            if p_end < (stop_distance+self.slack_distance):
                raise RuntimeError("fail due to lane end")
            else:
                a_brake_lane_end = min(a_brake_lane_end, self._max_accel(v_sense, 0, 0, p_end, delta_t))
                v_check = 0

        if first_obstacle_ahead is not None:
            # obstacle is closer than shadow
            obstacle, v_frenet, p_rel = first_obstacle_ahead
            FRS = translate_polygon(obstacle.calc_FRS(v_frenet, delta_t), p_rel-self.slack_distance)
            if not danger_zone.disjoint(FRS):
                raise RuntimeError("fail due to obstacle")
            else:
                print(v_sense, v_frenet, obstacle.a_max, p_rel - obstacle.bbox.ext_x/2, delta_t)
                print(self._max_accel(v_sense, v_frenet, obstacle.a_max, p_rel - obstacle.bbox.ext_x/2, delta_t))
                a_brake_obstacle = min(a_brake_obstacle)
                v_check = v_frenet - obstacle.a_max*delta_t
        print([a_brake_shadow, a_brake_obstacle, a_brake_lane_end])
        a_plan = min([a_brake_shadow, a_brake_obstacle, a_brake_lane_end])

        if first_intersection is not None:
            intersection_id, p_to_intersection = first_intersection # p_to_intersection is the distance from current position to intersection
            slow_ok, fast_ok, _, _, _ = self._is_safe_intersection(v_sense, t_sense, t_plan, intersection_id, 
                                                            p_to_intersection, obstacle_map, shadow_map)
            if not slow_ok and not fast_ok:
                raise RuntimeError("fail due to intersection")
            elif not fast_ok:
                a_plan = -self.a_max
            elif not slow_ok:
                if a_plan < self.a_max:
                    raise RuntimeError("fail due to slow down not safe")

        v_plan = v_sense+a_plan*delta_t
        p_plan = p_sense + v_sense*delta_t + 0.5*a_plan*delta_t**2
        l_plan = l_ego

        return p_plan, l_plan, v_plan, v_plan<v_check
            
    def _min_decelerate(self, v_ego, v_obs, a_max_obs, s_rel):
        #print(v_ego, v_obs, a_max_obs, s_rel)
        """
            Assume the vehicle ahead apply the maximum deceleration
            what is the minimum decelerate the ego vehicle need to apply to stop before collision
            s_rel is the relative distance between the fron of ego to the rear of the object
        """
        if v_ego == 0:
            return 0
        s_obs_stop = v_obs**2/(2*a_max_obs)
        s_ego_stop = s_obs_stop+s_rel-self.slack_distance
        a_ego_require = v_ego**2/(2*s_ego_stop)
        return -a_ego_require

    def _max_accel(self, v_ego, v_obs, a_max_obs, s_rel, delta_t):

        s_obs_stop = v_obs**2/(2*a_max_obs) + s_rel - self.slack_distance

        a = delta_t**2/(2*self.a_max)
        b = v_ego*delta_t/self.a_max+0.5*delta_t**2
        c = v_ego**2/(2*self.a_max)+v_ego*delta_t-s_obs_stop
        omega = np.sqrt(b**2-4*a*c)
        return (-b+omega)/(2*a)





        

        