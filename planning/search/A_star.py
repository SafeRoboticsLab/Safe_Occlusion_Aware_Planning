from os import P_ALL
from commonroad.common.validity import is_list_of_natural_numbers
import numpy as np
from queue import PriorityQueue, Queue
from risk_analysis import ISpace
from planning.lattice import Lattice, Node

from datetime import datetime



class A_star:
    def __init__(self, infoSpace: ISpace, lattice: Lattice, v_max, a_max_accel, a_max_decel, dt, T_plan):
        self.infoSpace = infoSpace
        self.lattice = lattice
        self.dt = dt
        self.v_max = v_max
        self.a_max_accel = a_max_accel
        self.a_max_decel = a_max_decel
        self.T_plan = T_plan # total time horizon
        self.frontier = None
        self.goal_node = None
        self.t0 = 0
        self.debug = False
        self.obstacle_predictor = self.infoSpace.object_manager.predict_obstacle

    def plan(self, p0, l0, v0, t0, goal_p, shadow_map, shadow_list, debug= False):       
        self.debug = debug

        self.infoSpace.reset_prediction()
    
        dt0 = datetime.now()
    
        self.goal_node = None

        self.frontier = PriorityQueue()
        self.add_node(p0, l0, v0, 0, None, 0, 0)
        self.t0 = t0

        minL = 0

        while not self.frontier.empty():
            cur_node = self.frontier.get()
            if self.lattice.add_node(cur_node):
                if cur_node.l<minL and self.debug:
                    print("Poped:", minL, cur_node.t)
                    minL = cur_node.l

                if goal_p is None:
                    if cur_node.t >= self.T_plan:
                        self.goal_node = cur_node
                        break
                elif cur_node.p>=goal_p and cur_node.l>=0:
                    print("Planner Reaching the goal")
                    self.goal_node = cur_node
                    break
                elif cur_node.p>=goal_p and cur_node.l<0:
                    continue
                elif cur_node.t >= self.T_plan:
                    print("Planner Reaching the timestep limit")
                    self.goal_node = cur_node
                    break
                
                if self.debug:
                    print("poped ", cur_node.p, cur_node.l, cur_node.v, cur_node.t)
                ego_pose = self.lattice.lattice_2_global(cur_node.p, cur_node.l)
                ego_loc = [ego_pose.location.x, ego_pose.location.y, ego_pose.location.z+1.8]
                
                if cur_node.parent is None:
                    cur_node.add_predict_shadow(shadow_list)
                    self.expand_node(cur_node, shadow_map)
                else:
                    # make an optimistic prediction of objects around
                    cur_t = cur_node.t + self.t0
                    old_shadow = cur_node.parent.predicted_shadows
                    shadow_map, shadow_list = self.infoSpace.predict(cur_t, old_shadow, ego_loc, use_record=True)
                    cur_node.add_predict_shadow(shadow_list)
                    self.expand_node(cur_node, shadow_map)
                #input("wait")
        
        process_time = datetime.now() - dt0
        print("Takes ", process_time.total_seconds(), " sec to plan")
    
    def plan_openloop(self, p0, l0, v0, t0, goal_p, shadow_map, debug= False):

        self.debug = debug

        self.infoSpace.reset_prediction()
    
        dt0 = datetime.now()
    
        self.goal_node = None

        self.frontier = PriorityQueue()
        self.add_node(p0, l0, v0, 0, None, 0, 0)
        self.t0 = t0

        while not self.frontier.empty():
            cur_node = self.frontier.get()
            if self.lattice.add_node(cur_node):
                if goal_p is None:
                    if cur_node.t >= self.T_plan:
                        self.goal_node = cur_node
                        break
                elif cur_node.p>=goal_p and cur_node.l>=-1:
                    print("Planner Reaching the goal")
                    self.goal_node = cur_node
                    break
                elif cur_node.p>=goal_p and cur_node.l<-1:
                    continue
                elif cur_node.t >= self.T_plan:
                    print("Planner Reaching the timestep limit")
                    self.goal_node = cur_node
                    break
                #print("poped ", cur_node.p, cur_node.l, cur_node.v, cur_node.t)                
                self.expand_node_openloop(cur_node, shadow_map)
                #input("wait")
        
        process_time = datetime.now() - dt0
        print("Takes ", process_time.total_seconds(), " sec to plan")
            
    def expand_node_openloop(self, cur_node, shadow_map):
        t_cur = cur_node.t + self.t0
        v_cur = cur_node.v
        p_cur = cur_node.p
        l_cur = cur_node.l
        t_plan = t_cur + self.dt
                        
        def max_lateral_a(dl):
            return (4*np.abs(dl))/(self.dt**2)
        
        def min_max_progress(a_max_accel, a_max_decel):
            # calculate admissible traj length
            if a_max_decel == 0:
                t2minV = self.dt
            else:
                t2minV = min(self.dt, v_cur/a_max_decel)
            
            min_dp = v_cur*t2minV-0.5*a_max_decel*t2minV**2

            if a_max_accel == 0:
                t2maxV = self.dt
            else:
                t2maxV = min(self.dt, max(0,(self.v_max - v_cur)/a_max_accel))
            max_dp = v_cur*t2maxV+0.5*a_max_accel*t2maxV**2+self.v_max*max(0, self.dt-t2maxV)
            return min_dp, max_dp

        dl_max = self.a_max_decel*(self.dt**2)/4
        l_idx_max = int(np.floor((dl_max+l_cur)/self.lattice.dl))
        l_idx_min = int(np.ceil((l_cur - dl_max)/self.lattice.dl))
        
        for l_idx in range(l_idx_min, l_idx_max+1):
            l_plan = l_idx*self.lattice.dl
            delta_l = l_plan-l_cur
            a_lat = max_lateral_a(delta_l)
            a_lon_decel_max = np.sqrt(self.a_max_decel**2 - a_lat**2)
            a_lon_accel_max = min(self.a_max_accel, a_lon_decel_max)
            min_dp, max_dp = min_max_progress(a_lon_accel_max, a_lon_decel_max)
            num_dp_min = int(np.ceil(min_dp/self.lattice.dp))
            num_dp_max = int(np.floor(max_dp/self.lattice.dp))
        
            for num_dp in range(num_dp_min, num_dp_max+1):
                delta_p = num_dp*self.lattice.dp
                p_plan = p_cur+delta_p
                # make sure this is an possible waypoint
                if (np.abs(delta_l)<=3*delta_p) and self.lattice.lattice_exist(p_plan, l_plan):
                    # assume constant acceleration during the step of planning
                    a_plan = (delta_p - v_cur*self.dt)/(0.5*self.dt**2)
                    v_plan = np.round(v_cur+a_plan*self.dt,4)
                    if v_plan<0 or v_plan>self.v_max:
                        continue
                    issafe, cross_cost, opposite_cost =  self.infoSpace.state_checker.is_safe_plan(p_plan, l_plan, v_plan, 0, self.t0, t_plan, 
                                                        self.obstacle_predictor, shadow_map, use_record = True, debug = self.debug)
                    if issafe:
                        self.add_node(p_plan, l_plan, v_plan, cur_node.t+self.dt, cur_node, cross_cost, opposite_cost)
                            
    def expand_node(self, cur_node,  shadow_map):
        t_cur = cur_node.t + self.t0
        v_cur = cur_node.v
        p_cur = cur_node.p
        l_cur = cur_node.l
        t_plan = t_cur + self.dt

                        
        def max_lateral_a(dl):
            return (4*np.abs(dl))/(self.dt**2)
        
        def min_max_progress(a_max_accel, a_max_decel):
            # calculate admissible traj length
            if a_max_decel == 0:
                t2minV = self.dt
            else:
                t2minV = min(self.dt, v_cur/a_max_decel)
            
            min_dp = v_cur*t2minV-0.5*a_max_decel*t2minV**2

            if a_max_accel == 0:
                t2maxV = self.dt
            else:
                t2maxV = min(self.dt, max(0,(self.v_max - v_cur)/a_max_accel))
            max_dp = v_cur*t2maxV+0.5*a_max_accel*t2maxV**2+self.v_max*max(0, self.dt-t2maxV)
            return min_dp, max_dp

        dl_max = self.a_max_decel*(self.dt**2)/4
        l_idx_max = int(np.floor((dl_max+l_cur)/self.lattice.dl))
        l_idx_min = int(np.ceil((l_cur - dl_max)/self.lattice.dl))
        
        for l_idx in range(l_idx_min, l_idx_max+1):
            l_plan = l_idx*self.lattice.dl
            delta_l = l_plan-l_cur
            a_lat = max_lateral_a(delta_l)
            a_lon_decel_max = np.sqrt(self.a_max_decel**2 - a_lat**2)
            a_lon_accel_max = min(self.a_max_accel, a_lon_decel_max)
            min_dp, max_dp = min_max_progress(a_lon_accel_max, a_lon_decel_max)
            num_dp_min = int(np.ceil(min_dp/self.lattice.dp))
            num_dp_max = int(np.floor(max_dp/self.lattice.dp))
            if self.debug:
                print("delta_l = :", delta_l, min_dp, max_dp, num_dp_min, num_dp_max)
        
            for num_dp in range(num_dp_min, num_dp_max+1):
                delta_p = num_dp*self.lattice.dp
                p_plan = p_cur+delta_p
                # make sure this is an possible waypoint
                if self.debug:
                    pass
                if (np.abs(delta_l)<=3*delta_p) and self.lattice.lattice_exist(p_plan, l_plan):
                    # assume constant acceleration during the step of planning
                    a_plan = (delta_p - v_cur*self.dt)/(0.5*self.dt**2)
                    v_plan = v_cur+a_plan*self.dt
                    if v_plan<0 or v_plan>self.v_max:
                        continue
                    issafe, cross_cost, opposite_cost = self.infoSpace.state_checker.is_safe_plan(p_plan, l_plan, v_plan, 0, t_cur, t_plan, 
                                                        self.obstacle_predictor, shadow_map, use_record = True, debug = self.debug)
                    if issafe:
                        self.add_node(p_plan, l_plan, v_plan, cur_node.t+self.dt, cur_node, cross_cost, opposite_cost)
            
    def add_node(self, p, l, v, t, parent, cross_cost, opposite_cost):
        if parent is None:
            cost2come = 0
        else:
            cost_turn = abs(l-parent.l)
            # cost_off_center = abs(l%3.6-1.8)
            # cost_opposite_lane = 2*(l<=0)
            cost2come = parent.cost2come + 0*cost_turn + 0.02*cross_cost + 0.04*opposite_cost
        p_round = np.round(p, 4)
        l_round = np.round(l, 4)

        # calculate cost2go
        v_max = self.v_max+self.a_max_accel*self.dt*0.5
        t_remain = self.T_plan - t
        t2maxV = min(t_remain, max(0,(v_max - v)/self.a_max_accel))
        cost2go = -p-(v*t2maxV + 0.5*self.a_max_accel*t2maxV**2 + v_max*max(0,(t_remain-t2maxV)))
        #print(p, l, v, t,-cost2go-cost2come)  
        node = Node(p_round, l_round, t, v, parent, cost2come, cost2go)
        self.frontier.put(node)

    def get_plan(self, dt_subsample):
        if self.goal_node is None:
            return None # no solution
        traj = {}
        cur_node = self.goal_node
        while cur_node.parent is not None:
            t_sampled, p_sampled, l_sampled, v_sampled, vl_sampled = self._subsample_traj(cur_node, dt_subsample)
            for t, p, l, v, vl in zip(t_sampled, p_sampled, l_sampled, v_sampled, vl_sampled):
                traj[t] = (p, l, v, vl, cur_node.p, cur_node.l, cur_node.v, 0, cur_node.t+self.t0)            
            cur_node = cur_node.parent
        return traj

    def _subsample_traj(self, cur_node, dt_subsample):
        prev_node = cur_node.parent
        t_sampled = np.round(np.flip(np.arange(self.dt, 0, -dt_subsample)), 4)

        ## longitudinal trajectory
        a_lon = (cur_node.v - prev_node.v)/self.dt

        v_sampled = prev_node.v+t_sampled*a_lon
        p_sampled = prev_node.p+prev_node.v*t_sampled + 0.5*a_lon*t_sampled**2

        ## lateral trajectory
        a_lat = 4*(cur_node.l -prev_node.l)/(self.dt**2)
        idx_accel = t_sampled <= (self.dt/2)
        l_sampled = np.ones_like(t_sampled)
        vl_sampled = np.ones_like(t_sampled)

        vl_sampled[idx_accel] = a_lat*t_sampled[idx_accel]
        vl_sampled[~idx_accel] = a_lat*(self.dt - t_sampled[~idx_accel])
        l_sampled[idx_accel] = prev_node.l+0.5*a_lat*t_sampled[idx_accel]**2
        l_sampled[~idx_accel] = cur_node.l - 0.5*a_lat*(self.dt - t_sampled[~idx_accel])**2

        t_sampled += prev_node.t + self.t0
        t_sampled = np.round(t_sampled,2)

        return t_sampled, p_sampled, l_sampled, v_sampled, vl_sampled

        
        
        


            
        
        
