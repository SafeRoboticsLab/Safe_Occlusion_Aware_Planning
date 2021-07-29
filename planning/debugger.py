import copy
from risk_analysis.shadow import InOutBound
from risk_analysis import ISpace
from planning import Lattice, InteractivePlanner
from .evasive_planner import Evasion
from client import SynchronousClient
import numpy as np
from matplotlib import pyplot as plt


class Debugger():
    def __init__(self, client: SynchronousClient, octomap: str, ego_path: list,
                    v_max = 20, a_max = 6, sensor_range = 100, dp = 0.5, dl = 0.5, dt_plan = 0.5, dv = 0.5, T_plan = 5, T_replan = 2.5):

        self.client = client
        
        self.dt_plan = dt_plan

        self.replan_period = T_replan

        self.info_space = ISpace(self.client, octomap, ego_path, 
                            range = sensor_range, ds = 0.5, v_max=v_max, a_max=a_max)
        self.lattice = Lattice(self.info_space.route, dp = dp, dl = dl, dv = dv,
                                dt = dt_plan, V_range=[0, v_max], t_range = [0, T_plan])
        self.planner = InteractivePlanner(self.info_space, self.lattice, v_max = v_max, a_max = a_max, dt = dt_plan, T_plan = T_plan)
        self.obstacle_predictor = self.info_space.object_manager.predict_obstacle
        
        self.planned_traj = None
        self.planned_node = None
        self.predicted_shadow = None

        self.t_cur = 0
        self.t_replan = None

        self.p_cur = None
        self.l_cur = None
        self.v_cur = None
        self.vl_cur = None

       
        

    def initialize_game(self, p0, l0, v0):

        self.lattice.reset(p0)
        self.t_replan = 0
        self.t_cur = 0
                
        self.p_cur = p0
        self.l_cur = l0
        self.vl_cur = 0
        self.v_cur = v0
        self.move_ego()
       
    def tick(self, sensor_data, sensor_pose):
        
        # Update the information space based on sensor observation
        shadow_map, shadow_list = self.info_space.update(self.t_cur, sensor_data, sensor_pose)
        

        if self.t_cur >= self.t_replan:
            print("replanning")
            self.lattice.reset(self.p_cur)
            self.planner.init(self.p_cur, self.l_cur, self.v_cur, self.t_cur, shadow_map, shadow_list)
            self.t_replan = self.t_cur+self.replan_period
                
        else:
            self.planner.plan_interactive()# retrive planned trajectory

        p_next, l_next, v_next, vl_next, t_next, self.predicted_shadow = self.planner.get_plan()
                   
        print("Move to (", p_next, l_next, v_next, vl_next, t_next,")")
        self.p_cur = p_next
        self.l_cur = l_next
        self.v_cur = v_next          
        self.vl_cur = vl_next
        self.t_cur = t_next
        
        self.move_ego()
    
   
    def move_ego(self):
        spwan_point = self.lattice.lattice_2_global(self.p_cur, self.l_cur)
        self.client.ego.set_transform(spwan_point)






        