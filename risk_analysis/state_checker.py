from os import sendfile

from commonroad.scenario.lanelet import Lanelet
from client import SynchronousClient
from mapping import RoadNetwork
from planning import Route
from risk_analysis.shadow_utils import  ShadowUtils
from risk_analysis.reachable_set import double_integrator_stop_dangerzone as stop_dangerzone
from risk_analysis.reachable_set import translate_polygon
from risk_analysis.reachable_set import double_integrator_reachable_tube as reachable_tube
from risk_analysis.reachable_set import double_integrator_merge_dangerzone as lane_change_dangerzone
import numpy as np
from queue import Queue
import heapq
import carla
from shapely.geometry import LineString, Polygon

import matplotlib.pyplot as plt

class StateChecker():
    def __init__(self, client:SynchronousClient, road_network: RoadNetwork, route: Route, shadow_manager: ShadowUtils, v_max, a_max):
        
        self.client = client
        self.shadow_manager = shadow_manager
        self.road_network = road_network
        self.route = route
        self.v_max = v_max#*1.2 # allow other agent to be 20% over speed limit
        self.a_max = a_max
        self.ego_width = 2
        self.ego_length = 5
        self.debug = False
        self.debugger = self.client.world.debug
        self.slack_distance = 2.5
        self.search_stop = 300

    def is_safe_plan(self, p_check, l_check, v_check, vl_check, t_sense, t_check, obstacle_predictor,  shadow_map_sense, use_record = False, debug=False):
        """
        Check if the ego object can safely move from ()_sense to ()_plan, given the predicted obstacle_map at t_check, shadow_map captured at t_sense 
        """
        self.debug = debug
        
        nominal_lanelet_id_check, ds_check = self.route.find_nominal_lane(p_check)
        lanelet_id_check, direction_check = self.route.find_lanelet_id(nominal_lanelet_id_check, l_check, self.ego_width)
        if lanelet_id_check is None:
            return False
        elif len(lanelet_id_check) == 1 and direction_check[0]:
            # the vehicle is entirely in ego lane
            issafe, _, _, _, _, _ = self._plan_close_loop_ego(lanelet_id_check[0], ds_check, p_check, l_check, v_check, vl_check, 
                                             t_sense, t_check, obstacle_predictor,  shadow_map_sense, use_record)
        elif len(lanelet_id_check) == 1 and not direction_check[0]:
            # the vehicle is entirely in the opposite lane
            section_check = self.route.get_route_section(p_check)
            # Assume drive on the right side of road
            lanelet_id_target, l_left_target, l_right_target = section_check.get_neighbor(l_check, True)
            dl_enter = l_left_target - l_check - self.ego_width/2
            dl_shield = l_left_target - l_check + self.ego_width/2
            dl_finish = l_right_target - l_check - self.ego_width/2
        
            issafe, _, _, _, _, _ = self._plan_close_loop_lane_change(lanelet_id_check[0], lanelet_id_target, ds_check, False, p_check, v_check,
                                                        l_check, vl_check, 1, dl_enter, dl_shield, dl_finish, t_sense, t_check, 
                                                        obstacle_predictor, shadow_map_sense, use_record)
        else:
            # cover two lanes
            left_lanelet_id = lanelet_id_check[0]
            right_lanelet_id = lanelet_id_check[1]
            left_direction = direction_check[0]
            right_direction = direction_check[1]
            section_check = self.route.get_route_section(p_check)
           
            # shielding to the right lane   
            if right_direction:         
                l_bound, r_bound = section_check.get_boundary(right_lanelet_id)
                dl_enter = l_bound - l_check - self.ego_width/2
                dl_shield = l_bound - l_check + self.ego_width/2
                dl_finish = r_bound - l_check - self.ego_width/2
                shield_right,_, _, _, _, _ = self._plan_close_loop_lane_change(left_lanelet_id, right_lanelet_id, ds_check, left_direction, p_check, v_check,
                                                            l_check, vl_check, 1, dl_enter, dl_shield, dl_finish, t_sense, t_check, 
                                                            obstacle_predictor, shadow_map_sense, use_record)
            else:
                shield_right = False
            
            if left_direction:
                # shielding to the left lane
                l_bound, r_bound = section_check.get_boundary(left_lanelet_id)
                dl_enter = r_bound - l_check - self.ego_width/2
                dl_shield = r_bound - l_check + self.ego_width/2
                dl_finish = l_bound - l_check - self.ego_width/2
                shield_left,_, _, _, _, _ = self._plan_close_loop_lane_change(right_lanelet_id, left_lanelet_id, ds_check, right_direction, p_check, v_check,
                                                        l_check, vl_check, -1, dl_enter, dl_shield, dl_finish, t_sense, t_check, 
                                                        obstacle_predictor, shadow_map_sense, use_record)
            else:
                shield_left = False
            issafe = shield_left or shield_right

        return issafe
            
    def plan_close_loop(self, p_cur, l_cur, v_cur, vl_cur, t_cur, t_plan, obstacle_predictor, shadow_map_cur, use_record = False, debug = False):

        """
        Check if the ego object can safely planned to (nominal_lanelet_id, ds, dl) at t_check based on observation at t_cur
        """
        # first determine the state
        self.debug = debug

        nominal_lanelet_id, ds = self.route.find_nominal_lane(p_cur)
        lanelet_id, direction = self.route.find_lanelet_id(nominal_lanelet_id, l_cur, self.ego_width)

        plan_dt = t_plan - t_cur
               
        if len(lanelet_id) == 1 and direction[0]:
            # occuping only one ego lane
            issafe, p_plan, l_plan, v_plan, vl_plan, is_shield = self._plan_close_loop_ego(lanelet_id[0], ds, p_cur, l_cur,
                                                                             v_cur, vl_cur, t_cur, t_cur, obstacle_predictor,  
                                                                             shadow_map_cur, use_record, plan_step = plan_dt)
            
        elif len(lanelet_id) == 1 and not direction[0]:
            # the vehicle is entirely in the opposite lane
            section_check = self.route.get_route_section(p_cur)
            # Assume drive on the right side of road
            lanelet_id_target, l_left_target, l_right_target = section_check.get_neighbor(l_cur, True)
            dl_enter = l_left_target - l_cur - self.ego_width/2
            dl_shield = l_left_target - l_cur + self.ego_width/2
            dl_finish = l_right_target - l_cur - self.ego_width/2
        
            issafe, p_plan, l_plan, v_plan, vl_plan, is_shield  = self._plan_close_loop_lane_change(lanelet_id[0], lanelet_id_target, ds, False, p_cur, v_cur,
                                                        l_cur, vl_cur, 1, dl_enter, dl_shield, dl_finish, t_cur, t_cur, 
                                                        obstacle_predictor, shadow_map_cur, use_record)
        else:
            # cover two lanes
            left_lanelet_id = lanelet_id[0]
            right_lanelet_id = lanelet_id[1]
            left_direction = direction[0]
            right_direction = direction[1]
            section_check = self.route.get_route_section(p_cur)
            
            # shielding to the right lane   
            if right_direction:         
                l_bound, r_bound = section_check.get_boundary(right_lanelet_id)
                dl_enter = l_bound - l_cur - self.ego_width/2
                dl_shield = l_bound - l_cur + self.ego_width/2
                dl_finish = r_bound - l_cur - self.ego_width/2
                shield_right, p_plan_right, l_plan_right, v_plan_right, vl_plan_right, is_shield_right  = self._plan_close_loop_lane_change(left_lanelet_id, right_lanelet_id, ds, left_direction, p_cur, v_cur,
                                                            l_cur, vl_cur, 1, dl_enter, dl_shield, dl_finish, t_cur, t_cur, 
                                                            obstacle_predictor, shadow_map_cur, use_record)
            else:
                shield_right = False
            
            if left_direction:
                # shielding to the left lane
                l_bound, r_bound = section_check.get_boundary(left_lanelet_id)
                dl_enter = r_bound - l_cur - self.ego_width/2
                dl_shield = r_bound - l_cur + self.ego_width/2
                dl_finish = l_bound - l_cur - self.ego_width/2
                shield_left, p_plan_left, l_plan_left, v_plan_left, vl_plan_left, is_shield_left  = self._plan_close_loop_lane_change(right_lanelet_id, left_lanelet_id, ds, right_direction, p_cur, v_cur,
                                                        l_cur, vl_cur, -1, dl_enter, dl_shield, dl_finish, t_cur, t_cur, 
                                                        obstacle_predictor, shadow_map_cur, use_record)
            else:
                shield_left = False
            issafe = shield_left or shield_right

            if shield_left and shield_right:
                if v_plan_left>v_plan_right:
                    p_plan = p_plan_left
                    l_plan = l_plan_left
                    v_plan = v_plan_left
                    vl_plan = vl_plan_left
                    is_shield = is_shield_left
                else:
                    p_plan = p_plan_right
                    l_plan = l_plan_right
                    v_plan = v_plan_right
                    vl_plan = vl_plan_right
                    is_shield = is_shield_right
            elif shield_left:
                p_plan = p_plan_left
                l_plan = l_plan_left
                v_plan = v_plan_left
                vl_plan = vl_plan_left
                is_shield = is_shield_left
            elif shield_right:
                p_plan = p_plan_right
                l_plan = l_plan_right
                v_plan = v_plan_right
                vl_plan = vl_plan_right
                is_shield = is_shield_right
            
                            
        if not issafe:
                raise RuntimeError("No close loop solution")

        return p_plan, l_plan, v_plan, vl_plan, is_shield

    def _plan_close_loop_ego(self, lanelet_id_check, ds_check, p_check, l_check, v_check, vl_check, t_sense, t_check, 
                                                            obstacle_predictor, shadow_map_sense, use_record, plan_step = 0.1):
        """ check if the ego stay in the same lane"""

        delta_t = t_check-t_sense

        danger_zone, stop_distance = stop_dangerzone(v_check, self.a_max, self.v_max, x_extend=self.ego_length/2)
        if self.debug:
            print("in _plan_close_loop_ego", stop_distance)
        
        # predict where obstacles are
        obstacle_map_sense, occupied_lanelet_sense = obstacle_predictor(t_sense, use_record)

        first_obj, first_shadow, first_intersection, lane_end = self._explore_fwd(lanelet_id_check, 
                                                                    ds_check, t_check, t_sense, obstacle_predictor, obstacle_map_sense, 
                                                                    occupied_lanelet_sense, shadow_map_sense, use_record)                                                  

        issafe = True
        
        a_brake_shadow = self.a_max
        a_brake_obj = self.a_max
        a_brake_lane_end = self.a_max

        v_shield = self.v_max # if planned the velocity is less than this, the ego vehicle can plan openloop again

        if first_shadow is not None:
            _, p_rel = first_shadow 
            p_remain = p_rel - self.ego_length/2 - self.slack_distance 
            if stop_distance>p_remain:
                # winning condition for close loop game is not satisief for t_check
                if self.debug:
                    print("not able to stop before shadow")
                issafe = False
            else:
                # plan one step for the close loop game from t_check
                if self.debug:
                    print(first_shadow, p_rel)
                a_brake_shadow = min(-self.a_max/2, self._max_accel(v_check, 0, 0, p_remain-1, plan_step))
                v_shield = 0
        
        if lane_end is not None:
            _, p_rel = lane_end
            p_remain = p_rel - self.ego_length/2 - self.slack_distance 
            if stop_distance>p_remain:
                if self.debug:
                    print("not able to stop before lane end")
                issafe = False
            else:
                a_brake_lane_end = min(-self.a_max/2, self._max_accel(v_check, 0, 0, p_remain-1, plan_step))
                v_shield = 0

        if first_obj is not None:
            # obstacle is closer than shadow
            
            obstacle, v_frenet, p_rel = first_obj
            p_shift = p_rel - self.slack_distance
            FRS = translate_polygon(obstacle.calc_FRS(v_frenet, delta_t), p_shift)
            if self.debug:
                plt.plot(*danger_zone.exterior.xy)
                if isinstance(FRS, Polygon):
                    plt.plot(*FRS.exterior.xy)
                else:
                    plt.plot(*FRS.xy)
                plt.show()
                print(first_shadow, p_rel)
            if not danger_zone.disjoint(FRS):
                if self.debug:
                    print("not able to stop before obstacle")
                issafe = False
            else:
                if isinstance(FRS, LineString):
                    p_remain = min(FRS.xy[0]) - self.ego_length/2 - 2
                else:
                    p_remain = min(FRS.exterior.xy[0]) - self.ego_length/2 - 2
                a_brake_obj = min(-self.a_max/2, self._max_accel(v_check, v_frenet, obstacle.a_max, p_remain, plan_step))
                if self.debug:
                    print("Brake",  p_remain, a_brake_obj)
                v_shield = v_frenet - obstacle.a_max*plan_step

        a_plan = max(-self.a_max, min([a_brake_shadow, a_brake_obj, a_brake_lane_end]))

        if first_intersection is not None:
            intersection_id, p_to_intersection = first_intersection # p_to_intersection is the distance from current position to intersection
            p_to_intersection = p_to_intersection
            slow_ok, fast_ok, v_terminal, t_terminal, p_rel_terminal = self._is_safe_intersection(v_check, t_sense, t_check, intersection_id, 
                                                            p_to_intersection, obstacle_map_sense, shadow_map_sense)
            if not slow_ok and not fast_ok:
                if self.debug:
                    print("slow and fast both impossible")
                issafe = False
            elif not fast_ok:
                a_plan = -self.a_max
            elif not slow_ok:
                p_exit = p_check+p_rel_terminal
                safe_exit = self.is_safe_plan(p_exit, l_check, v_terminal, 0, t_sense, t_terminal, obstacle_predictor, shadow_map_sense, use_record, self.debug)
                if not safe_exit:
                    issafe = False
                else:
                    a_plan = self.a_max
            else:
                # both slow and quick is ok
                a_plan = min(a_plan,0)
        else:
            a_plan = min(a_plan,0)

        a_lon = a_plan
        a_lat_max = np.sqrt(self.a_max**2 - a_lon**2)
        if vl_check != 0:
            a_lat = -np.sign(vl_check)*min(vl_check/plan_step, a_lat_max)
            vl_plan = vl_check+a_lat*plan_step
            l_plan = l_check+vl_check*plan_step+0.5*a_lat*plan_step**2          
        else:
            l_plan = l_check
            vl_plan = 0
            a_lat = 0
               
        v_plan = v_check+a_lon*plan_step
        if v_plan > self.v_max and a_lon>0:
            t_2_max = (self.v_max - v_check)/a_lon
            p_plan = p_check + v_check*t_2_max + 0.5*a_lon*t_2_max**2+self.v_max*(plan_step - t_2_max)
            v_plan = self.v_max
        elif v_plan<0:
            t_2_stop = v_check/a_lon
            p_plan = p_check + v_check*t_2_stop + 0.5*a_lon*t_2_stop**2
            v_plan = 0
        else:
            p_plan = p_check + v_check*plan_step + 0.5*a_lon*plan_step**2
        
        vl_plan = np.round(vl_plan, 4)
        v_plan = np.round(v_plan, 4)
        
        is_shield = (v_plan <= v_shield) and (vl_plan == 0)
        if self.debug:
            print(v_plan, v_shield)
        
        return issafe, p_plan, l_plan, v_plan, vl_plan, is_shield
        
    def _explore_fwd(self, lanelet_id, ds, t_check, t_sense, obstacle_predictor, obstacle_map_sense, 
                                                            occupied_lanelet_sense, shadow_map_sense, use_record):
        
        """
        From (lanelet_id, ds) search forward, find the closest obstacle, shadow, intersection and lane end ahead within the stop distance
        at the time of sense
        """
        
        obstacle_map_check, _ = obstacle_predictor(t_check, use_record)
        delta_t = t_check - t_sense
        
        cur_lane = self.road_network.find_lane_id(lanelet_id)
        cur_lanelet_id = cur_lane.lanelet_id
        p_covered = -cur_lane.delta_s_from_begin(ds) 

        first_obj = None
        first_shadow = None
        first_intersection = None

        first_merged_shadow = None
        first_merged_obj = None

        lane_end = None
        p_rel_obs = np.inf
        # get an idea of what is ahead
        while p_covered < self.search_stop:
            if first_obj is None:
                if cur_lanelet_id in obstacle_map_check:
                    cur_obj_queue = obstacle_map_check[cur_lanelet_id]
                    if cur_lane.lane_id<0: 
                        # the closest object have smaller ds
                        sorted_obj = heapq.nsmallest(len(cur_obj_queue), cur_obj_queue)
                    else: 
                        # the closest object have larger ds
                        sorted_obj = heapq.nlargest(len(cur_obj_queue), cur_obj_queue)
                    for obs_ds, v_frenet, obstacle in sorted_obj:
                        p_rel = cur_lane.delta_s_from_begin(obs_ds)+p_covered
                        p_rel_front = p_rel + 0.5*obstacle.a_max*delta_t
                        if p_rel_front>0:
                            obs_id = obstacle.id
                            lanelet_sense = occupied_lanelet_sense[obs_id]
                            same_lane = False
                            for lanelet in lanelet_sense:
                                if self.route.in_ego_path(lanelet):
                                    same_lane = True
                                    break
                            if same_lane:
                                first_obj = (obstacle, v_frenet, p_rel-v_frenet*delta_t)
                                break
                            

            if first_shadow is None:
                # Find the first shadow ahead. If it is intersected with danger zone, retun 2, distance to shadow
                if cur_lanelet_id in shadow_map_sense:
                    cur_shadow_queue = shadow_map_sense[cur_lanelet_id]
                    if cur_lane.lane_id<0: 
                        # the closest object have smaller ds
                        sorted_shadow = heapq.nsmallest(len(cur_shadow_queue), cur_shadow_queue)
                    else: 
                        # the closest object have larger ds
                        sorted_shadow = heapq.nlargest(len(cur_shadow_queue), cur_shadow_queue)

                    if cur_lanelet_id in obstacle_map_sense:
                        cur_obj_queue = obstacle_map_sense[cur_lanelet_id]
                        if cur_lane.lane_id<0: 
                            # the closest object have smaller ds
                            sorted_obj = heapq.nsmallest(len(cur_obj_queue), cur_obj_queue)
                        else: 
                            # the closest object have larger ds
                            sorted_obj = heapq.nlargest(len(cur_obj_queue), cur_obj_queue)
                        # make sure the shadow is not right before the vehicle

                        for obs_ds, v_frenet, obstacle in sorted_obj:
                            p_rel = cur_lane.delta_s_from_begin(obs_ds)+p_covered
                            if p_rel < 0:
                                continue
                            else:
                                # if self.debug:
                                #     print("old obstacle has :", p_rel, cur_lanelet_id, obs_ds, v_frenet)
                                p_rel_obs = p_rel-obstacle.bbox.ext_x
                                break

                    for shadow_ds, shadow in sorted_shadow:
                        # TODO how about shadow merge into the current lane
                        p_rel = cur_lane.delta_s_from_begin(shadow_ds)+p_covered
                        # if self.debug:
                        #     print("found shadow:", p_rel, cur_lanelet_id, shadow_ds)
                        # consider the worst case for the shadow is some static obstacle
                        if p_rel <= 0:
                            # this only occur at the lane that ego vehicle occupied
                            continue
                        elif p_rel < p_rel_obs:
                            first_shadow = (shadow, p_rel)
                            break
                       
            if first_intersection is None:
                # check if current lane is an intersection, and the car can break before intersection
                if cur_lane.cross is not None:
                    first_intersection = (cur_lanelet_id, p_covered)

            p_covered += cur_lane.length

            if p_covered < self.search_stop:
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

        return first_obj, first_shadow, first_intersection, lane_end

    def _explore_bwd(self, lanelet_id, ds, t_check, t_sense, obstacle_map_sense, shadow_map_sense):

        delta_t = t_check - t_sense
        cur_lane = self.road_network.find_lane_id(lanelet_id)
        cur_lanelet_id = lanelet_id
        p_covered = -cur_lane.delta_s_from_end(ds) 

        obj_list = []
        shadow_list = []

        lane_queue = Queue()
        lane_queue.put((lanelet_id, p_covered))

        while not lane_queue.empty():
            cur_lanelet_id, processed_ds = lane_queue.get()
            cur_lane = self.road_network.find_lane_id(cur_lanelet_id)

            # first check if there is an object 
            if cur_lanelet_id in obstacle_map_sense:
                cur_obj_queue = obstacle_map_sense[cur_lanelet_id]
                if cur_lane.lane_id>0: 
                    # the closest object have smaller ds
                    sorted_obj = heapq.nsmallest(len(cur_obj_queue), cur_obj_queue)
                else: 
                    # the closest object have larger ds
                    sorted_obj = heapq.nlargest(len(cur_obj_queue), cur_obj_queue)
                for obs_ds, v_frenet, obstacle in sorted_obj:
                    p_rel = cur_lane.delta_s_from_end(obs_ds)+processed_ds-v_frenet*delta_t
                    if p_rel >0:
                        obj_list.append((obstacle, v_frenet, p_rel))
                                                          
            if cur_lanelet_id in shadow_map_sense:
                cur_shadow_queue = shadow_map_sense[cur_lanelet_id]
                if cur_lane.lane_id<0: 
                    # the closest object have smaller ds
                    sorted_shadow = heapq.nsmallest(len(cur_shadow_queue), cur_shadow_queue)
                else: 
                    # the closest object have larger ds
                    sorted_shadow = heapq.nlargest(len(cur_shadow_queue), cur_shadow_queue)
                for shadow_ds, shadow in sorted_shadow:
                    p_rel = cur_lane.delta_s_from_end(shadow_ds)+processed_ds
                    if p_rel > 0:
                        shadow_list.append((shadow, p_rel))
                        
                    
            processed_ds += cur_lane.length
            if processed_ds < self.search_stop:
                for predecessor in cur_lane.predecessor:
                    lane_queue.put((predecessor, processed_ds))
        
        return obj_list, shadow_list

    def _is_safe_intersection(self, v_check, t_sense, t_check, intersection_id, p_to_intersection, obstacle_map_sense, shadow_map_sense):
        """
        intersection_lane is the lane the ego vehicle will drive
        p_to_intersection from the p_check to the begin of the intersection lane
        """

        intersection_lane = self.road_network.find_lane_id(intersection_id)
        

        # distance to the place when ego vehicle first enter an intersection

        risk_lane_list = []
        
        # first iterate through all cross and calculate the distance of enter and exit this intersection from current point
        
        for risky_lane_id in intersection_lane.cross:
            # lane that cross the intersection_lane
            risky_lane = self.road_network.find_lane_id(risky_lane_id) 
            
            risk_ds = intersection_lane.get_cross_ds(risky_lane_id, risky_lane) 
            # relative distance bewteen the ego center to the place where the front of ego vehicle will enter intersection
            p_in_rel = intersection_lane.delta_s_from_begin(risk_ds[0])+p_to_intersection - self.ego_length/2
            # relative distance bewteen the ego center to the place where the back of ego vehicle will leave intersection
            p_out_rel =intersection_lane.delta_s_from_begin(risk_ds[1])+p_to_intersection + self.ego_length/2
            heapq.heappush(risk_lane_list, (p_in_rel, p_out_rel, risky_lane))
            

        risk_lane_list = heapq.nsmallest(len(risk_lane_list), risk_lane_list)

        
        

        p_enter_rel = risk_lane_list[0][0]
        
        slow_ok = True
        fast_ok = True
        v_terminal = 0
        t_terminal = 0
        p_rel_terminal = 0
        stop_dis = v_check**2/(2*self.a_max)
        if self.debug:
            print("stop distance: ", stop_dis, "first intersection", p_enter_rel)
        if stop_dis<p_enter_rel:
            if self.debug:
                print("Stop before intersection", stop_dis, p_enter_rel)
            return True, True, v_terminal, t_terminal, p_rel_terminal

        if risk_lane_list[-1][1] <=0.5:
            """ pass all intersection """
            v_terminal = v_check
            t_terminal = t_terminal
            p_rel_terminal = 0
            return True, True, v_terminal, t_terminal, p_rel_terminal
        elif p_enter_rel>=0:
            """ if the ego has not enter the intersection """
            t_enter_slow, v_enter_slow = self.time2finish(v_check, -self.a_max, p_enter_rel, self.v_max)
            t_enter_fast, v_enter_fast = self.time2finish(v_check, self.a_max, p_enter_rel, self.v_max)
            for p_in_rel, p_out_rel, risky_lane in risk_lane_list:
                if self.debug:
                    print("check risky lane before intersection:", risky_lane.lanelet_id)
                t_slow_in, v_slow_in =   self.time2finish(v_enter_slow, self.a_max, 
                                                p_in_rel - p_enter_rel, self.v_max)+t_enter_slow
                t_slow_out, v_slow_out = self.time2finish(v_enter_slow, self.a_max, 
                                                p_out_rel - p_enter_rel, self.v_max)+t_enter_slow

                t_fast_in, v_fast_in =   self.time2finish(v_enter_fast, self.a_max, 
                                                p_in_rel - p_enter_rel, self.v_max)+t_enter_fast
                t_fast_out, v_fast_out = self.time2finish(v_enter_fast, self.a_max, 
                                                p_out_rel - p_enter_rel, self.v_max)+t_enter_fast
                #print(t_enter_slow, t_enter_fast, t_slow_in, t_slow_out, t_fast_in, t_fast_out)
                if t_fast_out>t_terminal:
                    t_terminal = t_fast_out
                    v_terminal = v_fast_out
                    p_rel_terminal = p_out_rel
                # ds of riskylane where the ego path goes through
                cross_ds = risky_lane.get_cross_ds(intersection_id, intersection_lane)
                ego_lane_width = np.abs(cross_ds[0]-cross_ds[1])

                BRT_slow = reachable_tube(0, self.v_max, t_slow_in, t_slow_out, self.a_max, 0, self.v_max, ego_lane_width/2, BRS=True)
                BRT_fast = reachable_tube(0, self.v_max, t_fast_in, t_fast_out, self.a_max, 0, self.v_max, ego_lane_width/2, BRS=True)
                
                # plt.plot(*BRT_slow.exterior.xy)
                # plt.plot(*BRT_fast.exterior.xy)
                # plt.show()

                results = self._check_intersection_bwd(risky_lane, (cross_ds[0]+cross_ds[1])/2, t_sense, t_check, 
                                                        [BRT_slow, BRT_fast], obstacle_map_sense, shadow_map_sense)
                if not results[0]:
                    slow_ok = False
                if not results[1]:
                    fast_ok = False
                if not slow_ok and not fast_ok:
                    return slow_ok, fast_ok, v_terminal, t_terminal, p_rel_terminal
            if self.debug:
                print("enter intersection", slow_ok, fast_ok ) 
        else:
            slow_ok = False
            for p_in_rel, p_out_rel, risky_lane in risk_lane_list:
                if self.debug:
                    print("check risky lane in intersection:", risky_lane.lanelet_id)
                if p_out_rel<0:
                    continue
                elif p_in_rel<0:
                    # planned trajectory is crossing this lane
                    t_out, v_out = self.time2finish(v_check, self.a_max, 
                                                p_out_rel , self.v_max)
                    # ds of riskylane where the ego path goes through
                    cross_ds = risky_lane.get_cross_ds(intersection_id, intersection_lane)
                    ego_lane_width = np.abs(cross_ds[0]-cross_ds[1])
                    danger_zone = reachable_tube(0, self.v_max, 0, t_out, self.a_max, 0, self.v_max, ego_lane_width/2, BRS=True)
                    
                else:
                    t_in, v_in =   self.time2finish(v_check, self.a_max, 
                                                p_in_rel , self.v_max)
                    t_out, v_out = self.time2finish(v_check, self.a_max, 
                                                p_out_rel , self.v_max)
                    cross_ds = risky_lane.get_cross_ds(intersection_id, intersection_lane)
                    ego_lane_width = np.abs(cross_ds[0]-cross_ds[1])
                    danger_zone = reachable_tube(0, self.v_max, t_in, t_out, self.a_max, 0, self.v_max, ego_lane_width/2, BRS=True)
                results = self._check_intersection_bwd(risky_lane, (cross_ds[0]+cross_ds[1])/2, t_sense, t_check, 
                                                    [danger_zone], obstacle_map_sense, shadow_map_sense)
                if t_out>t_terminal:
                    t_terminal = t_out
                    v_terminal = v_out
                    p_rel_terminal = p_out_rel
                fast_ok = results[0]
                if not fast_ok:
                    return slow_ok, fast_ok, v_terminal, t_terminal, p_rel_terminal
        return slow_ok, fast_ok, v_terminal, t_terminal, p_rel_terminal

    def _check_intersection_bwd(self, lane, ds, t_sense, t_check, danger_zone_list, obstacle_map_sense, shadow_map_sense):
        if self.debug:
            print("calling _check_intersection_bwd")
        delta_t = t_check - t_sense

        dis_safe_list = [min(danger_zone.exterior.xy[0]) for danger_zone in danger_zone_list]
        dis_safe = min(dis_safe_list)- 2*delta_t*self.v_max

        processed_ds = -lane.delta_s_from_end(ds)
        lanelet_id = lane.lanelet_id

        lane_queue = Queue()
        lane_queue.put((lanelet_id, processed_ds))

        results_list = [True for _ in danger_zone_list]

        while not lane_queue.empty():
            cur_lanelet_id, processed_ds = lane_queue.get()
            cur_lane = self.road_network.find_lane_id(cur_lanelet_id)

            # first check if there is an object 
            if cur_lanelet_id in obstacle_map_sense:
                cur_obj_queue = obstacle_map_sense[cur_lanelet_id]
                if cur_lane.lane_id>0: 
                    # the closest object have smaller ds
                    sorted_obj = heapq.nsmallest(len(cur_obj_queue), cur_obj_queue)
                else: 
                    # the closest object have larger ds
                    sorted_obj = heapq.nlargest(len(cur_obj_queue), cur_obj_queue)
                for obs_ds, v_frenet, obstacle in sorted_obj:
                    #p_shift = p_rel - self.slack_distance - v_frenet*delta_t
                    p_shift = cur_lane.delta_s_from_end(obs_ds)+processed_ds+self.slack_distance
                    FRS = translate_polygon(obstacle.calc_FRS(v_frenet, delta_t), -p_shift)
                    if self.debug:
                        print("FRS shift", -p_shift, v_frenet, cur_lanelet_id, cur_lane.delta_s_from_end(obs_ds), processed_ds)
                                              
                    for i, danger_zone in enumerate(danger_zone_list):
                        
                        if not FRS.disjoint(danger_zone):
                            results_list[i] = False
                    
                    if self.debug and True in results_list:
                        print(results_list)
                        for zone in danger_zone_list:
                            plt.plot(*zone.exterior.xy)
                        if isinstance(FRS, LineString):
                            plt.plot(*FRS.xy, '--')
                        else:
                            plt.plot(*FRS.exterior.xy, '--')
                        plt.show()

            
            if cur_lanelet_id in shadow_map_sense:
                cur_shadow_queue = shadow_map_sense[cur_lanelet_id]
                if cur_lane.lane_id<0: 
                    # the closest object have smaller ds
                    sorted_shadow = heapq.nsmallest(len(cur_shadow_queue), cur_shadow_queue)
                else: 
                    # the closest object have larger ds
                    sorted_shadow = heapq.nlargest(len(cur_shadow_queue), cur_shadow_queue)
                for shadow_ds, shadow in sorted_shadow:
                    p_rel = cur_lane.delta_s_from_end(shadow_ds)+processed_ds
                    FRS = self.shadow_manager.bwd_shadow_FRS(shadow, t_sense, t_check)
                    if FRS is None:
                        #print(shadow.root.id, shadow.root.ds_out, shadow.leaf_depth, "FRS is None")
                        continue
                    FRS = translate_polygon(FRS, -p_rel)
                                           
                    for i, danger_zone in enumerate(danger_zone_list):
                        if not FRS.disjoint(danger_zone):
                            results_list[i] = False
                        # if self.debug:
                        #     plt.plot(*danger_zone.exterior.xy)
                    # if self.debug:
                    #     plt.plot(*FRS.exterior.xy, '*')
                    #     plt.show()
                        
                    
            processed_ds += cur_lane.length
            if processed_ds < -dis_safe:
                for predecessor in cur_lane.predecessor:
                    lane_queue.put((predecessor, processed_ds))

        return results_list

    def _plan_close_loop_lane_change(self, lanelet_id_check, lanelet_id_target, ds_check, direction_check, p_check, v_check, l_check, vl_check, lateral_dir,
                                                            dl_enter, dl_shield, dl_finish, t_sense, t_check,
                                                            obstacle_predictor, shadow_map_sense, use_record, plan_step = 0.1 ):

        delta_t = t_check - t_sense
        obstacle_map_sense, occupied_lanelet_sense = obstacle_predictor(t_sense, use_record)        
        gap_list = self._find_lane_change_gaps(lanelet_id_target, ds_check, t_sense, t_check, obstacle_predictor, occupied_lanelet_sense,shadow_map_sense, use_record)
        
        # HACK end
        

        dt_finish = np.sqrt(4*dl_finish/self.a_max)
        if dl_shield < dl_finish/2:
            dt_shield = np.sqrt(2*dl_shield/self.a_max)
            dt_finish = 2*dt_shield
        else:
            dt_shield = dt_finish - np.sqrt(2*(dl_finish-dl_shield)/self.a_max)
        
        if dl_enter < 0:
            dt_enter = 0
        elif dl_enter< dl_finish/2:
            dt_enter = np.sqrt(4*dl_enter/self.a_max)
        else: # dl_shield>dl_enter>1/2dl_finish
            dt_enter = dt_finish - np.sqrt(2*(dl_finish-dl_enter)/self.a_max)

        if self.debug:
            print("in _plan_close_loop_lane_change move from {} to {}".format(lanelet_id_check, lanelet_id_target))
            print("dl_enter = {}, dl_shield = {}, dl_finish = {}".format(dl_enter, dl_shield, dl_finish))
            print("dt_enter = {}, dt_shield = {}, dt_finish = {}".format(dt_enter, dt_shield, dt_finish))

        """ Get first object ahead"""
        if direction_check:
            # check lane is the in the same direction
            object_list, shadow_list, _, _ = self._explore_fwd(lanelet_id_check, ds_check, t_check, t_sense, obstacle_predictor, 
                                                                    obstacle_map_sense, occupied_lanelet_sense, shadow_map_sense, use_record)
        else: 
            # check lane is the opposite direction
            object_list, shadow_list = self._explore_bwd(lanelet_id_check, ds_check, t_check, t_sense, obstacle_map_sense, shadow_map_sense)

        """check each gap"""
        find_safe_gap = False
        fastest_t_shield = np.inf
        action = None
        for gap in gap_list:
            t_start_lane_change, t_shield, v_shield, dp_shield = self._lane_change_to_gap(v_check, gap, dt_enter, dt_shield, dt_finish)
            if self.debug:
                print("t_start_lane_change={}, t_shield={}, v_shield={}, dp_shield={}".format(t_start_lane_change, t_shield, v_shield, dp_shield))
            if t_start_lane_change is None:
                continue
            
            danger_zone = lane_change_dangerzone(dp_shield, t_shield, self.a_max, self.v_max, self.ego_length/2)
            if self.debug:
                plt.plot(*danger_zone.exterior.xy)
            ok_to_lane_change = True
            if direction_check:
                if object_list is not None:
                    obstacle, v_frenet, dp_obj = object_list # at time sense
                    FRS = translate_polygon(obstacle.calc_FRS(v_frenet, delta_t), dp_obj)
                    if self.debug:
                        if isinstance(FRS, Polygon):
                            plt.plot(*FRS.exterior.xy)
                        else:
                            plt.plot(*FRS.xy)
                        print("outgoing obstacle at ", dp_obj, v_frenet)
                    if not danger_zone.disjoint(FRS):
                        ok_to_lane_change  = False
                if shadow_list is not None:
                    _, dp_rel = shadow_list
                    if self.debug:
                        print("outgoing shadow at ", dp_rel)
                    if dp_rel < dp_shield:
                        ok_to_lane_change  = False
            else:
                for obj in object_list:
                    obstacle, v_frenet, dp_obj = obj # at time sense
                    if self.debug:
                        FRS = translate_polygon(obstacle.calc_FRS(v_frenet, delta_t,True), dp_obj)
                        if isinstance(FRS, Polygon):
                            plt.plot(*FRS.exterior.xy)
                        else:
                            plt.plot(*FRS.xy)
                        print("incoming obstacle at ", dp_obj, v_frenet)
                    if not danger_zone.disjoint(FRS):
                        ok_to_lane_change  = False
                        break
                if ok_to_lane_change:
                    for shadow_tuple in shadow_list:
                        shadow, dp_shadow = shadow_tuple
                        FRS = self.shadow_manager.bwd_shadow_FRS(shadow, t_sense, t_check, oppo=True)
                        
                        FRS = translate_polygon(FRS, dp_shadow)
                        if self.debug:
                            if isinstance(FRS, Polygon):
                                plt.plot(*FRS.exterior.xy)
                            else:
                                plt.plot(*FRS.xy)
                            print("incoming shadow at ", dp_shadow)
                        if FRS is None:
                            continue
                        if not danger_zone.disjoint(FRS):
                            ok_to_lane_change  = False
                            break
            if self.debug:
                plt.show()
            if ok_to_lane_change:
                    find_safe_gap = True
                    if t_shield<fastest_t_shield:
                        # determine action
                        if t_check<t_start_lane_change:
                            # full acceleration/deceleration
                            action = (np.sign(v_shield - v_check)*self.a_max, 0)
                        else:
                            lateral_stop = vl_check**2/(2*self.a_max)
                            if lateral_stop<dl_finish:
                                action = (0, self.a_max)
                            else:
                                action = (0, -self.a_max)
        

        if action is not None:
            v_plan = v_check+action[0]*plan_step
            if v_plan > self.v_max and action[0]!=0:
                t_2_max = (self.v_max - v_check)/action[0]
                p_plan = p_check + v_check*t_2_max + 0.5*action[0]*t_2_max**2+self.v_max*(plan_step - t_2_max)
                v_plan = self.v_max
            elif v_plan<0 and action[0]!=0:
                t_2_stop = v_check/action[0]
                p_plan = p_check + v_check*t_2_stop + 0.5*action[0]*t_2_stop**2
                v_plan = 0
            else:
                p_plan = p_check + v_check*plan_step + 0.5*action[0]*plan_step**2
            
            
            vl_plan = vl_check + action[1]*plan_step
            l_plan = l_check+lateral_dir*(vl_check*plan_step+0.5*action[1]*plan_step**2)
        else: 
            p_plan=l_plan=v_plan=vl_plan = None
           
        return find_safe_gap, p_plan, l_plan, v_plan, vl_plan, False
                        
    def _lane_change_to_gap(self, v_check, gap, dt_enter, dt_shield, dt_finish):
        dp_front, v_front, _, dp_back, v_back, _ = gap
        delta_p_front = dp_front - self.ego_length/2
        delta_p_back = dp_back + self.ego_length/2

        if self.debug:
            print("In gap ", gap, "to the front of gap", delta_p_front, "to the end of gap", delta_p_back)

        if dp_front - dp_back <= self.ego_length:
            if self.debug:
                print("gap too tight")
                return None, None, None, None

        """ determine when the ego is going to be parallel with the gap """
        if delta_p_back>0:
            # the ego vehicle is behind the gap
            if v_back>=self.v_max:
                if self.debug:
                    ("v back is too fast, cannot catch")
                return None, None, None, None
            t_catch_up, v_catch_up, dp_catch_up = self._catch_up_accel(delta_p_back, v_check, v_back)
        elif delta_p_front<0: 
            # the ego vehicle is ahead of the gap
            if v_front < 2:
                if self.debug:
                    ("v front is too slow, cannot catch")
                return None, None, None, None
            t_catch_up, v_catch_up, dp_catch_up = self._catch_up_decel(delta_p_front, v_check, v_front)
        else:
            t_catch_up = 0
            v_catch_up = v_check
            dp_catch_up = 0
            
        if self.debug:
            print("t_catch_up={}, v_catch_up={}, dp_catch_up={}".format(t_catch_up, v_catch_up, dp_catch_up))
        """ After being parallel with the gap, determine when is safe to do lateral motion"""
        t_start_lane_change, v_start_lane_change, dp_start_lane_change = self._lane_change_in(t_catch_up, v_catch_up, dp_catch_up, gap, dt_enter, dt_finish)

        """ Calculate when and wher the ego vehicle will completely shild in the target gap"""
        if t_start_lane_change is not None:
            t_shield = t_start_lane_change + dt_shield
            v_shield = v_start_lane_change
            dp_shield = dp_start_lane_change + v_shield*dt_shield
        else:
            t_start_lane_change=t_shield=v_shield=dp_shield = None


        return t_start_lane_change, t_shield, v_shield, dp_shield

    def _catch_up_accel(self, delta_p_back, v_check, v_bound_back):
        # Full acceleration to catch up to back bound of gap
        a = 0.5*self.a_max
        b = v_check - v_bound_back
        c = -delta_p_back # < 0
        omega = np.sqrt(b**2-4*a*c)
        t_catch_up = (-b+omega)/(2*a)
        v_catch_up = v_check+t_catch_up*self.a_max
        p_catch_up = v_check*t_catch_up+0.5*self.a_max*t_catch_up**2
        if v_catch_up > self.v_max:
            t_to_max = (self.v_max - v_check)/self.a_max
            delta_p_2 = delta_p_back+v_bound_back*t_to_max - v_check*t_to_max - 0.5*self.a_max*t_to_max**2
            t_catch_up = t_to_max + delta_p_2/(self.v_max - v_bound_back)
            v_catch_up = self.v_max
            p_catch_up = v_check*t_to_max+0.5*self.a_max*t_to_max**2+v_catch_up*(t_catch_up - t_to_max)
        return t_catch_up, v_catch_up, p_catch_up
    
    def _catch_up_decel(self, delta_p_front, v_check, v_bound_front):
        # Full deceleration to catch up to back bound of gap
        a = -0.5*self.a_max
        b = v_check - v_bound_front
        c = -delta_p_front # > 0
        omega = np.sqrt(b**2-4*a*c)
        t_catch_up = (-b+omega)/(2*a)
        v_catch_up = v_check+t_catch_up*self.a_max
        p_catch_up = v_check*t_catch_up+0.5*self.a_max*t_catch_up**2
        v_min = 2
        if v_catch_up < v_min:
            t_to_min = (v_check-v_min)/self.a_max
            delta_p_2 = delta_p_front+v_bound_front*t_to_min - v_check*t_to_min + 0.5*self.a_max*t_to_min**2
            t_catch_up = t_to_min + delta_p_2/(v_min - v_bound_front)
            v_catch_up = v_min
            p_catch_up = v_check*t_to_min-0.5*self.a_max*t_to_min**2+v_catch_up*(t_catch_up - t_to_min)
        return t_catch_up, v_catch_up, p_catch_up

    def _lane_change_in(self, t_catch_up, v_catch_up, dp_catch_up, gap, dt_enter, dt_finish):
        dt_response = 0.5
        delta_p_front, v_front, a_front, dp_back, v_back, a_back = gap
        """ ASSUMPTION: vehicle in target lane will keep constant velocity until the ego vehicle enter the target lane"""
        """ based on the time of catching up, check if the ego can start lateral motion or keep accelerate/decelerate"""
        # relative distance to the front boundary of gap. Vehicle should be behind this bound
        # delta_p means the relative distance
        # dp is the distance ego travel
        delta_p_front_catch_up = delta_p_front+v_front*t_catch_up - dp_catch_up - self.ego_length/2
        delta_p_front_enter = delta_p_front_catch_up+dt_enter*(v_front - v_catch_up)

        # distance to the back boundary of gap. Vehicle should be ahead this bound
        delta_p_back_catch_up = dp_back+v_back*t_catch_up - dp_catch_up + self.ego_length/2 # this value should <=0
        delta_p_back_enter = delta_p_back_catch_up+ dt_enter*(v_back - v_catch_up)

        # the vehicle in the front brake at the time of entering, but the ego cannot respond until finish lane change
        danger_zone_front = max(0, v_catch_up**2/(2*self.a_max)+(dt_finish-dt_enter)*v_catch_up - v_front**2/(2*a_front))
        danger_zone_back = max(0, v_back*dt_response + v_back**2/(2*a_back) - v_catch_up**2/(2*self.a_max))
        if self.debug:
            print("delta_p_front_enter ={}, delta_p_back_enter = {}, danger_zone_front= {}, danger_zone_back={}".format(delta_p_front_enter, delta_p_back_enter, danger_zone_front, danger_zone_back))

        v_min = 2
        if delta_p_front_enter >= danger_zone_front and (-delta_p_back_enter) >= danger_zone_back:
            dt_lat_start = t_catch_up
            v_lat_start = v_catch_up
            dp_lat_start = dp_catch_up
        elif delta_p_front_enter < danger_zone_front and (-delta_p_back_enter) < danger_zone_back:
            if self.debug:
                print("gap is too tight")
            return None, None, None
        elif delta_p_front_enter < danger_zone_front:
            if v_front <= v_min:
                if self.debug:
                    print("No way to slow down v_front: ",v_front)
                return None, None, None
            num = v_catch_up**2/(2*self.a_max) - v_front**2/(2*a_front) - delta_p_front_catch_up + v_catch_up*dt_finish - v_front*dt_enter
            det = v_front + self.a_max*dt_finish
            dt_exceed = num/det
            v_exceed = v_catch_up - self.a_max*dt_exceed
            dp_exceed = v_catch_up*dt_exceed - 0.5*self.a_max*dt_exceed**2
            if v_exceed<v_min:
                t_to_min = (v_catch_up - v_min)/self.a_max
                p_to_min = v_catch_up*t_to_min - 0.5*self.a_max*t_to_min**2
                lane_change_safe_distance = v_min**2/(2*self.a_max) - v_front**2/(2*a_front)
                dis_remain = lane_change_safe_distance + p_to_min - delta_p_front_catch_up - v_front*(t_to_min+dt_enter) +v_min*dt_finish
                dt_exceed = t_to_min + dis_remain / (v_front - v_min)
                v_exceed = v_min
                dp_exceed = p_to_min + v_min*dt_exceed
            dt_lat_start = t_catch_up+dt_exceed
            v_lat_start = v_exceed
            dp_lat_start = dp_catch_up+dp_exceed            
        else:
            if v_back>=self.v_max:
                if self.debug:
                    print("No way to speed up v_back: ",v_back)
                return None, None, None
            a = self.a_max
            b = 2*v_catch_up+self.a_max*dt_enter - v_back
            c = v_catch_up*dt_enter - delta_p_back_catch_up + v_catch_up**2/(2*self.a_max) - v_back**2/(2*a_back) - v_back*dt_response
            omega = np.sqrt(b**2-4*a*c) 
            dt_exceed = (-b+omega)/(2*a)
            v_exceed = v_catch_up+dt_exceed*self.a_max
            dp_exceed = v_catch_up*dt_exceed+0.5*self.a_max*dt_exceed**2
            if v_exceed > self.v_max:
                t_to_max = (self.v_max - v_catch_up)/self.a_max
                p_to_max = v_catch_up*t_to_max + 0.5*self.a_max*t_to_max**2
                lane_change_safe_distance = v_back*dt_response + v_back**2/(2*a_back) - self.v_max**2/(2*self.a_max)
                dis_remain =  lane_change_safe_distance + v_back*t_to_max + delta_p_back_catch_up - p_to_max + (v_back - self.v_max)*dt_enter
                dt_exceed = t_to_max + dis_remain/(self.v_max - v_back)
                v_exceed = self.v_max
                dp_exceed = p_to_max + v_exceed*(dt_exceed - t_to_max)
            dt_lat_start = t_catch_up+dt_exceed
            v_lat_start = v_exceed
            dp_lat_start = dp_catch_up+dp_exceed
        
            

        """ based on the time of lateral start, check if the ego can be safe at time of enter"""


        delta_p_frondt_lat_start = delta_p_front + v_front*dt_lat_start - dp_lat_start - self.ego_length/2
        delta_p_front_enter = delta_p_frondt_lat_start+dt_enter*(v_front - v_lat_start)
        delta_p_back_lat_start = dp_back+v_back*dt_lat_start - dp_lat_start + self.ego_length/2 # this value should <=0
        delta_p_back_enter = delta_p_back_lat_start+ dt_enter*(v_back - v_lat_start)

        danger_zone_front = max(0, v_lat_start**2/(2*self.a_max)+(dt_finish-dt_enter)*v_catch_up - v_front**2/(2*a_front))
        danger_zone_back = max(0, v_back*dt_response + v_back**2/(2*a_back) - v_lat_start**2/(2*self.a_max))


        if self.debug:
            print("dt_lat_start={}, v_lat_start={}, dp_lat_start={}".format(dt_lat_start, v_lat_start, dp_lat_start))

        if delta_p_front_enter < danger_zone_front or (-delta_p_back_enter) < danger_zone_back:
            if self.debug:
                print("After speed mathc, too tight, Not safe")
            return None, None, None
        else:
            return dt_lat_start, v_lat_start, dp_lat_start
                 
    def _find_lane_change_gaps(self, lanelet_id_target, ds_check, t_sense, t_check,  obstacle_predictor, occupied_lanelet_sense, shadow_map_sense, use_record):

        #HACK 
        # We are just trying to find the cloest obstacle
        # TODO:
        # find all gaps that the vehicle may lane_change in
        delta_t = t_check - t_sense
        #_, occupied_lanelet_sense = obstacle_predictor(t_sense, use_record)
        obstacle_map_check, _ = obstacle_predictor(t_check, use_record)

        """ explore forward"""
        self.search_stop = 3*self.v_max**2/(2*self.a_max)
        cur_lane = self.road_network.find_lane_id(lanelet_id_target)
        cur_lanelet_id = lanelet_id_target
        p_covered = -cur_lane.delta_s_from_begin(ds_check) 

        obstacle_cloest = None
        obstacle_behind = None
        obstacle_ahead = None
        
        while p_covered < self.search_stop and cur_lanelet_id is not None and obstacle_ahead is None:
            if cur_lanelet_id in obstacle_map_check:
                cur_obj_queue = obstacle_map_check[cur_lanelet_id]
                if cur_lane.lane_id<0: 
                    # the closest object have smaller ds
                    sorted_obj = heapq.nsmallest(len(cur_obj_queue), cur_obj_queue)
                else: 
                    # the closest object have larger ds
                    sorted_obj = heapq.nlargest(len(cur_obj_queue), cur_obj_queue)
                for obs_ds, v_frenet, obstacle in sorted_obj:
                    p_rel = cur_lane.delta_s_from_begin(obs_ds)+p_covered
                    if self.debug:
                        print("Found object on target lane", p_rel, obs_ds, v_frenet)
                    if obstacle_cloest is None:
                        obstacle_cloest = (p_rel, v_frenet, obstacle)
                    elif abs(p_rel)<abs(obstacle_cloest[0]):
                        obstacle_behind = obstacle_cloest
                        obstacle_cloest = (p_rel, v_frenet, obstacle)
                    else:
                        obstacle_ahead = (p_rel, v_frenet, obstacle)
                        break
            
            p_covered += cur_lane.length
            cur_lanelet_id = None
            for successor in cur_lane.successor:
                if self.route.in_ego_path(successor):
                    cur_lanelet_id = successor
                    cur_lane = self.road_network.find_lane_id(cur_lanelet_id)
                    break
            
        
        """ explore backward """
        p_covered = -cur_lane.delta_s_from_begin(ds_check)
        cur_lane = self.road_network.find_lane_id(lanelet_id_target)
        cur_lanelet_id = None
        for predecessor in cur_lane.predecessor:
            if self.route.in_ego_path(predecessor):
                cur_lanelet_id = predecessor
                cur_lane = self.road_network.find_lane_id(cur_lanelet_id)
                break

        while p_covered < self.search_stop and cur_lanelet_id is not None and obstacle_behind is None:
            if cur_lanelet_id in obstacle_map_check:
                cur_obj_queue = obstacle_map_check[cur_lanelet_id]
                if cur_lane.lane_id>0: 
                    # the closest object have smaller ds
                    sorted_obj = heapq.nsmallest(len(cur_obj_queue), cur_obj_queue)
                else: 
                    # the closest object have larger ds
                    sorted_obj = heapq.nlargest(len(cur_obj_queue), cur_obj_queue)
                for obs_ds, v_frenet, obstacle in sorted_obj:
                    p_rel = cur_lane.delta_s_from_end(obs_ds)+p_covered
                    if obstacle_cloest is None:
                        obstacle_cloest = (p_rel, v_frenet, obstacle)
                    elif abs(p_rel)<abs(obstacle_cloest[0]):
                        obstacle_ahead = obstacle_cloest
                        obstacle_cloest = (p_rel, v_frenet, obstacle)
                    else:
                        obstacle_behind = (p_rel, v_frenet, obstacle)
                        break
            
            p_covered += cur_lane.length
            cur_lanelet_id = None
            for predecessor in cur_lane.predecessor:
                if self.route.in_ego_path(predecessor):
                    cur_lanelet_id = predecessor
                    cur_lane = self.road_network.find_lane_id(cur_lanelet_id)
                    break

        """ form gaps"""
        gap_list = []
        if obstacle_cloest is None:
            gap_list.append((np.inf, 0 , self.a_max, -np.inf, self.v_max, self.a_max))
        else:
            """ add the gap ahead of cloest obstacle"""
            p_front = np.inf
            v_front = 0
            a_front = self.a_max
            p_rel_closest, v_closest, obstacle_closest = obstacle_cloest

            v_back = v_closest
            a_back = obstacle_closest.a_max
            # HACK assume constant veloity
            p_back = p_rel_closest + obstacle_closest.bbox.ext_x  +  self.slack_distance
            #p_back = p_rel_closest + a_back*delta_t+ 0.5*a_back*delta_t**2+obstacle_closest.bbox.ext_x+self.slack_distance
            if obstacle_ahead is None:
                # HACK ignore the merge case
                cur_lanelet_id = occupied_lanelet_sense[obstacle.id][0]
                cur_lane = self.road_network.find_lane_id(cur_lanelet_id)
                p_covered = -cur_lane.delta_s_from_begin(ds_check)
                found_shadow = False
                while p_covered < self.search_stop and not found_shadow and cur_lanelet_id is not None:
                    if cur_lanelet_id in shadow_map_sense:
                        cur_shadow_queue = shadow_map_sense[cur_lanelet_id]
                        if cur_lane.lane_id<0: 
                            # the closest object have smaller ds
                            sorted_shadow = heapq.nsmallest(len(cur_shadow_queue), cur_shadow_queue)
                        else: 
                            # the closest object have larger ds
                            sorted_shadow = heapq.nlargest(len(cur_shadow_queue), cur_shadow_queue)

                        for shadow_ds, shadow in sorted_shadow:
                            if self.debug:
                                print("find shadow in ", shadow.root.id)
                            # TODO how about shadow merge into the current lane
                            p_shadow_start = cur_lane.delta_s_from_begin(shadow_ds)+p_covered
                            if None in shadow.leaf_depth:
                                p_shadow_end = np.inf
                                v_shadow = self.v_max
                            else:
                                shadow_depth = 0
                                v_shadow = 0
                                for in_bound, depth in zip(shadow.leaf_list, shadow.leaf_depth):
                                    if self.route.in_ego_path(in_bound.id):
                                        shadow_depth = depth
                                        if depth >(3+self.slack_distance):
                                            if self.debug:
                                                print(in_bound.t_in_hist, in_bound.t_in)
                                            v_shadow = max(v_back, self.shadow_manager.calc_V_max(in_bound, depth-3-self.slack_distance, t_sense))
                                        else:
                                            v_shadow = v_back
                                        break
                                p_shadow_end = p_shadow_start+shadow_depth+v_shadow*delta_t

                            if self.debug:
                                print("starts at", shadow_ds, p_shadow_start, p_shadow_end, v_shadow, shadow.leaf_depth, p_back)

                            if p_shadow_end < p_back:
                                continue
                            elif p_shadow_start>p_back:
                                p_front = p_shadow_start
                                a_front = self.a_max
                                v_front = 0
                                found_shadow = True
                            elif p_shadow_end == np.inf:
                                # the object drive into the shadow
                                p_front = p_back
                                a_front = a_back
                                v_front = v_back
                                found_shadow = True
                            else:
                                # there is a shadow in the front of cloest object
                                p_back = p_shadow_end
                                a_back = self.a_max
                                v_back = v_shadow
                                
                    p_covered += cur_lane.length
                    cur_lanelet_id = None
                    for successor in cur_lane.successor:
                        if self.route.in_ego_path(successor):
                            cur_lanelet_id = successor
                            cur_lane = self.road_network.find_lane_id(cur_lanelet_id)
                            break

            else:
                p_rel_front, v_front, obstacle_front = obstacle_ahead
                a_front = obstacle_front.a_max
                p_front = p_rel_front-obstacle_front.bbox.ext_x-self.slack_distance

            
            gap_list.append((p_front, v_front, a_front, p_back, v_back, a_back))

            """ add the gap behind the cloest obstacle"""
            p_rel_front, v_front, obstacle_front = obstacle_cloest
            a_front = obstacle_front.a_max
            p_front = p_rel_front - 0.5*a_front*delta_t**2-obstacle_front.bbox.ext_x-self.slack_distance
            
            if obstacle_behind is None:
                p_back = -np.inf
                v_back = self.v_max
                a_back = self.a_max
            else:
                p_rel_back, v_back, obstacle_back = obstacle_behind
                a_back = obstacle_back.a_max
                p_back = p_rel_back + 0.5*a_back*delta_t**2+obstacle_front.bbox.ext_x+self.slack_distance
            
            gap_list.append((p_front, v_front, a_front, p_back, v_back, a_back))

        return gap_list
            
    @staticmethod
    def time2finish(v0, a, dis, v_max):
        if dis == 0:
            return 0, v0

        # solve 0.5at^2+v0t-dis = 0
        coeff_a = 0.5*a
        coeff_b = v0
        coeff_c = -dis

        coeff_d = (coeff_b**2)-4*coeff_a*coeff_c
        
        t1 = (-coeff_b-np.sqrt(coeff_d))/(2*coeff_a)
        t2 = (-coeff_b+np.sqrt(coeff_d))/(2*coeff_a)

        if a<0:
            t = min(t1, t2)
            v_terminal = v0 +t*a
        else:
            t = max(t1, t2)
            # what if exceed the max 
            v_terminal = v0 +t*a
            if v_terminal>v_max:
                t2vmax = (v_max-v0)/a
                dis2vmax = v0*t2vmax+0.5*a*(t2vmax**2)
                dis_remain = dis - dis2vmax
                t = t2vmax+dis_remain/v_max
                v_terminal = v_max
        return t, v_terminal

    def _max_accel(self, v_ego, v_obs, a_max_obs, s_remain, plan_step):
        if a_max_obs == 0:
            s_obs_stop = s_remain
        else:
            s_obs_stop = v_obs**2/(2*a_max_obs) + s_remain

        a = plan_step**2/(2*self.a_max)
        b = v_ego*plan_step/self.a_max+0.5*plan_step**2
        c = v_ego**2/(2*self.a_max)+v_ego*plan_step-s_obs_stop

        if (b**2-4*a*c)<0 and self.debug:
            print("Warning, ", v_ego, v_obs, a_max_obs, s_remain)
        omega = np.sqrt(b**2-4*a*c)
        return (-b+omega)/(2*a)

    @staticmethod
    def _min_decelerate(v_ego, v_obs, a_max_obs, p_rel):
        """
            Assume the vehicle ahead apply the maximum deceleration
            what is the minimum decelerate the ego vehicle need to apply to stop before collision
            p_rel is the relative distance between the fron of ego to the rear of the object
        """
        if v_ego == 0:
            return 0
        s_obs_stop = v_obs**2/(2*a_max_obs)
        s_ego_stop = s_obs_stop+p_rel-3
        a_ego_require = v_ego**2/(2*s_ego_stop)
        return -a_ego_require

    def debug_plot(self, lanelet_id, ds):

        lane = self.road_network.find_lane_id(lanelet_id)
        s = lane.lane_section.s_sense + ds
        waypoint = self.client.map.get_waypoint_xodr(lane.road_id, lane.section_id, s)
        wpt_t = waypoint.transform
        begin = wpt_t.location + carla.Location(z=0.5)
        angle = np.radians(wpt_t.rotation.yaw)
        end = begin + carla.Location(x=np.cos(angle), y=np.sin(angle))
        self.debugger.draw_arrow(begin, end, arrow_size=0.3, life_time=1.0)

   
        
        




    


