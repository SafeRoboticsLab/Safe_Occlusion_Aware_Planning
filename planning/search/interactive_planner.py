from commonroad.common.validity import is_list_of_natural_numbers
import numpy as np
from queue import PriorityQueue, Queue
from risk_analysis import ISpace
from planning.lattice import Lattice, Node

from datetime import datetime



class InteractivePlanner:
    def __init__(self, infoSpace: ISpace, lattice: Lattice, v_max, a_max, dt, T_plan):
        self.infoSpace = infoSpace
        self.lattice = lattice
        self.dt = dt
        self.v_max = v_max
        self.a_max = a_max
        self.frontier = None
        self.t0 = 0
        self.debug = False
        self.obstacle_predictor = self.infoSpace.object_manager.predict_obstacle
        self.poped_node = []
        self.T_plan = T_plan

        self.shadow_map = None
        self.shadow_list = None
        self.active_node = None

    def init(self, p0, l0, v0, t0, shadow_map, shadow_list):
        self.debug = True

        self.infoSpace.reset_prediction()
        
        self.frontier = Queue()
        self.add_node(p0, l0, v0, 0, None)
        self.active_node.add_predict_shadow(self.shadow_list)
        self.t0 = t0
        self.poped_node = []

        self.shadow_map = shadow_map
        self.shadow_list = shadow_list



        # self.state_start = []
    def plan_interactive(self):
        
        cur_node = self.active_node
        add_node = self.expand_interactive(cur_node, self.shadow_map)
        
        if add_node:
            self.poped_node.append(cur_node)
                        
            ego_pose = self.lattice.lattice_2_global(self.active_node.p, self.active_node.l)
            ego_loc = [ego_pose.location.x, ego_pose.location.y, ego_pose.location.z+1.8]
        
            predict_t = self.active_node.t + self.t0
            old_shadow = self.shadow_list

            dt0 = datetime.now()
            self.shadow_map, self.shadow_list = self.infoSpace.predict(predict_t, old_shadow, ego_loc, use_record=True)
            process_time = datetime.now() - dt0
            print("Takes ", process_time.total_seconds(), " to predict")

            self.active_node.add_predict_shadow(self.shadow_list)
        else:
            self.active_node = self.poped_node.pop()
            ego_pose = self.lattice.lattice_2_global(self.active_node.p, self.active_node.l)
            ego_loc = [ego_pose.location.x, ego_pose.location.y, ego_pose.location.z+1.8]
            predict_t = self.active_node.t + self.t0
            old_shadow = self.active_node.parent.shadow_list
            self.shadow_map, self.shadow_list = self.infoSpace.predict(predict_t, old_shadow, ego_loc, use_record=True)


                
    
    def expand_interactive(self, cur_node,  shadow_map):
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

        dl_plan_available = (np.arange(-num_dl_max, num_dl_max+1)*self.lattice.dl).tolist()
        add_node = False

        while len(dl_plan_available)>0 and not add_node:
            for i, l in enumerate(dl_plan_available):
                print(i," - ",l)
            l_idx = int(input("Choose following dl:".format(p_cur, l_cur)))
            dl = dl_plan_available[l_idx]
            l_plan = dl+l_cur
            dl_plan_available.remove(dl)

            a_lat = max_lateral_a(dl)
            a_lon_max = np.sqrt(self.a_max**2 - a_lat**2)
            
            min_dp, max_dp = min_max_progress(a_lon_max)
            num_dp_min = int(np.ceil(min_dp/self.lattice.dp))
            num_dp_max = int(np.floor(max_dp/self.lattice.dp))
            print(a_lon_max, min_dp, max_dp, num_dp_min, num_dp_max)

            dp_plan_available = (np.arange(num_dp_min, num_dp_max+1)*self.lattice.dp).tolist()
            while len(dp_plan_available)>0 and not add_node:
                for i, p in enumerate(dp_plan_available):
                    print(i," - ",p)
                p_idx = int(input("Choose following dp:".format(p_cur, l_cur, v_cur, t_cur)))
                dp = dp_plan_available[p_idx]
                p_plan = dp+p_cur
                dp_plan_available.remove(dp)
                if self.lattice.lattice_exist(p_plan, l_plan):
                    # assume constant acceleration during the step of planning
                    a_plan = (dp - v_cur*self.dt)/(0.5*self.dt**2)
                    v_plan = v_cur+a_plan*self.dt
                    print("plane to p={}, l={}, v={}, t={}".format(p_plan, l_plan, v_plan, t_plan ))
                    if not self.infoSpace.state_checker.is_safe_plan(p_plan, l_plan, v_plan, 0, t_cur, t_plan, 
                                                        self.obstacle_predictor, shadow_map, use_record = True, debug = True):
                        print("not safe")
                    else:
                        self.add_node(p_plan, l_plan, v_plan, cur_node.t+self.dt, cur_node)
                        add_node = True
                        
                   
        return add_node
            
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
        self.active_node  =  Node(p_round, l_round, t, v, parent, cost2come, cost2go)

    def get_plan(self):
        p_next = self.active_node.p
        l_next = self.active_node.l
        v_next = self.active_node.v
        vl_next = 0
        t_next = self.active_node.t+self.t0
        shadow = self.shadow_list
        return p_next, l_next, v_next, vl_next, t_next, shadow

        
        
        


            
        
        
