from commonroad.common.validity import is_list_of_natural_numbers
import numpy as np
from queue import PriorityQueue, Queue
from risk_analysis import ISpace
from planning.lattice import Lattice, Node

from datetime import datetime



class A_star:
    def __init__(self, infoSpace: ISpace, lattice: Lattice, v_max, a_max, dt, T_plan):
        self.infoSpace = infoSpace
        self.lattice = lattice
        self.dt = dt
        self.v_max = v_max
        self.a_max = a_max
        self.T_plan = T_plan # total time horizon
        self.frontier = None
        self.goal_node = None
        self.t0 = 0
        self.debug = False
        self.obstacle_predictor = self.infoSpace.object_manager.predict_obstacle

    def plan(self, p0, l0, v0, t0, shadow_map, shadow_list, debug= False):       
        self.debug = debug

        self.infoSpace.reset_prediction()
    
        dt0 = datetime.now()
    
        self.goal_node = None

        self.frontier = PriorityQueue()
        self.add_node(p0, l0, v0, 0, None)
        self.t0 = t0

        while not self.frontier.empty():
            cur_node = self.frontier.get()
            if self.lattice.add_node(cur_node):
                if cur_node.t >= self.T_plan or cur_node.p >= self.lattice.p_list[-1]:
                    #print(cur_node.s, cur_node.cost2come, cur_node.cost2go, cur_node.v)
                    self.goal_node = cur_node
                    break
                #print("poped ", cur_node.p, cur_node.l, cur_node.v, cur_node.t)
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
    
    def plan_openloop(self, p0, l0, v0, t0, shadow_map, shadow_list, debug= False):

        self.debug = debug

        self.infoSpace.reset_prediction()
    
        dt0 = datetime.now()
    
        self.goal_node = None

        self.frontier = PriorityQueue()
        self.add_node(p0, l0, v0, 0, None)
        self.t0 = t0

        while not self.frontier.empty():
            cur_node = self.frontier.get()
            if self.lattice.add_node(cur_node):
                if cur_node.t >= self.T_plan or cur_node.p >= self.lattice.p_list[-1]:
                    #print(cur_node.s, cur_node.cost2come, cur_node.cost2go, cur_node.v)
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
            return (4*dl)/(self.dt**2)
        
        def min_max_progress(a_max):
            # calculate admissible traj length
            t2minV = min(self.dt, v_cur/self.a_max)
            min_dp = v_cur*t2minV-0.5*a_max*t2minV**2
            t2maxV = min(self.dt, max(0,(self.v_max - v_cur)/self.a_max))
            max_dp = v_cur*t2maxV+0.5*a_max*t2maxV**2+self.v_max*max(0, self.dt-t2maxV)
            return min_dp, max_dp

        dl_max = self.a_max*(self.dt**2)/4
        num_dl_max = int(np.floor(dl_max/self.lattice.dl))
        #print("max laterl motion", dl_max, num_dl_max)
        for num_dl in range(-num_dl_max, num_dl_max+1):
            l_plan = l_cur+num_dl*self.lattice.dl
            a_lat = max_lateral_a(num_dl*self.lattice.dl)
            a_lon_max = np.sqrt(self.a_max**2 - a_lat**2)
            min_dp, max_dp = min_max_progress(a_lon_max)
            num_dp_min = int(np.ceil(min_dp/self.lattice.dp))
            num_dp_max = int(np.floor(max_dp/self.lattice.dp))
            #print(l_plan)
        
            for num_dp in range(num_dp_min, num_dp_max+1):
                p_plan = p_cur+num_dp*self.lattice.dp
                # make sure this is an possible waypoint
                if self.lattice.lattice_exist(p_plan, l_plan):
                    # assume constant acceleration during the step of planning
                    a_plan = ((num_dp*self.lattice.dp) - v_cur*self.dt)/(0.5*self.dt**2)
                    v_plan = v_cur+a_plan*self.dt

                    if self.infoSpace.state_checker.is_safe_plan(p_plan, l_plan, v_plan, 0, self.t0, t_plan, 
                                                        self.obstacle_predictor, shadow_map, use_record = True, debug = self.debug):
                        self.add_node(p_plan, l_plan, v_plan, cur_node.t+self.dt, cur_node)
                            
    def expand_node(self, cur_node,  shadow_map):
        t_cur = cur_node.t + self.t0
        v_cur = cur_node.v
        p_cur = cur_node.p
        l_cur = cur_node.l
        t_plan = t_cur + self.dt

                        
        def max_lateral_a(dl):
            return (4*dl)/(self.dt**2)
        
        def min_max_progress(a_max):
            # calculate admissible traj length
            t2minV = min(self.dt, v_cur/self.a_max)
            min_dp = v_cur*t2minV-0.5*a_max*t2minV**2
            t2maxV = min(self.dt, max(0,(self.v_max - v_cur)/self.a_max))
            max_dp = v_cur*t2maxV+0.5*a_max*t2maxV**2+self.v_max*max(0, self.dt-t2maxV)
            return min_dp, max_dp

        dl_max = self.a_max*(self.dt**2)/4
        num_dl_max = int(np.floor(dl_max/self.lattice.dl))
        
        for num_dl in range(-num_dl_max, num_dl_max+1):
            delta_l =  num_dl*self.lattice.dl
            l_plan = l_cur+delta_l
            a_lat = max_lateral_a(delta_l)
            a_lon_max = np.sqrt(self.a_max**2 - a_lat**2)
            min_dp, max_dp = min_max_progress(a_lon_max)
            num_dp_min = int(np.ceil(min_dp/self.lattice.dp))
            num_dp_max = int(np.floor(max_dp/self.lattice.dp))
        
            for num_dp in range(num_dp_min, num_dp_max+1):
                delta_p = num_dp*self.lattice.dp
                p_plan = p_cur+delta_p
                # make sure this is an possible waypoint
                if (delta_l<=3*delta_p) and self.lattice.lattice_exist(p_plan, l_plan):
                    # assume constant acceleration during the step of planning
                    a_plan = (delta_p - v_cur*self.dt)/(0.5*self.dt**2)
                    v_plan = v_cur+a_plan*self.dt
                    if self.infoSpace.state_checker.is_safe_plan(p_plan, l_plan, v_plan, 0, t_cur, t_plan, 
                                                        self.obstacle_predictor, shadow_map, use_record = True, debug = self.debug):
                        self.add_node(p_plan, l_plan, v_plan, cur_node.t+self.dt, cur_node)
            
    def add_node(self, p, l, v, t, parent):
        if parent is None:
            cost2come = 0
        else:
            cost_turn = abs(l-parent.l)
            # cost_off_center = abs(l%3.6-1.8)
            # cost_opposite_lane = 2*(l<=0)
            cost2come = parent.cost2come + 0*cost_turn #+ 1*cost_off_center+cost_opposite_lane
        p_round = np.round(p, 4)
        l_round = np.round(l, 4)

        # calculate cost2go
        v_max = self.v_max+self.a_max*self.dt*0.5
        t_remain = self.T_plan - t
        t2maxV = min(t_remain, max(0,(v_max - v)/self.a_max))
        cost2go = -p-1*(v*t2maxV + 0.5*self.a_max*t2maxV**2 + v_max*max(0,(t_remain-t2maxV)))  
        node = Node(p_round, l_round, t, v, parent, cost2come, cost2go)
        self.frontier.put(node)

    def get_plan(self, dt_subsample):
        plan_node = {}
        traj = {}
        cur_node = self.goal_node
        while cur_node.parent is not None:
            plan_node[np.round(cur_node.t+self.t0, 2)] = cur_node
            t_sampled, p_sampled, l_sampled, v_sampled, vl_sampled = self._subsample_traj(cur_node, dt_subsample)
            for t, p, l, v, vl in zip(t_sampled, p_sampled, l_sampled, v_sampled, vl_sampled):
                traj[t] = (p, l, v, vl, cur_node.p, cur_node.l, cur_node.v, cur_node.t+self.t0)            
            cur_node = cur_node.parent
        return traj, plan_node

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

        
        
        


            
        
        
