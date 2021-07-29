import copy
from risk_analysis.shadow import InOutBound
from risk_analysis import ISpace
from planning import Lattice, A_star
from .evasive_planner import Evasion
from client import SynchronousClient
import numpy as np
from matplotlib import pyplot as plt


class HybridGame():
    def __init__(self, client: SynchronousClient, octomap: str, ego_path: list,
                    v_max = 20, a_max = 6, sensor_range = 100, dp = 0.5, dl = 0.5, dt_sim = 0.1, dt_plan = 0.5, dv = 0.5, T_plan = 5, T_replan = 0.5):

        self.octomap_filepath = octomap
        self.ego_path = ego_path
        self.sensor_range = sensor_range
        
        self.client = client
        
        self.dt_sim = dt_sim
        self.dt_plan = dt_plan

        self.replan_period = T_replan

        self.info_space = ISpace(self.client, octomap, ego_path, 
                            range = sensor_range, ds = 0.5, v_max=v_max, a_max=a_max)
        self.lattice = Lattice(self.info_space.route, dp = dp, dl = dl, dv = dv,
                                dt = dt_plan, V_range=[0, v_max], t_range = [0, T_plan])
        self.open_loop = A_star(self.info_space, self.lattice, v_max = v_max, a_max = a_max, dt = dt_plan, T_plan = T_plan)
        self.close_loop = self.info_space.state_checker.plan_close_loop
        self.safe_checker = self.info_space.state_checker.is_safe_plan
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

        self.plan_open_loop = True 
        
        self.t_hist = []
        self.p_hist = []
        self.l_hist = []
        self.v_hist = []
        

    def initialize_game(self, p0, l0, v0):

        self.lattice.reset(p0)
        self.t_replan = 0
        self.t_cur = 0
                
        self.p_cur = p0
        self.l_cur = l0
        self.vl_cur = 0
        self.v_cur = v0
        self.move_ego()
       
    def tick(self, sensor_data, sensor_pose, open_loop = False):
        
        t_next = np.round(self.t_cur + self.dt_sim, 4)
        # Update the information space based on sensor observation
        shadow_map, shadow_list = self.info_space.update(self.t_cur, sensor_data, sensor_pose)
        
        if self.plan_open_loop:
            # if replan is needed
            if self.t_cur >= self.t_replan:
                print("replanning")
                debug = False
                self.lattice.reset(self.p_cur)
                if open_loop:
                    self.open_loop.plan_openloop(self.p_cur, self.l_cur, self.v_cur, self.t_cur, shadow_map, shadow_list, debug)
                else:
                    self.open_loop.plan(self.p_cur, self.l_cur, self.v_cur, self.t_cur, shadow_map, shadow_list, debug)
                self.planned_traj, self.planned_node = self.open_loop.get_plan(self.dt_sim)
                self.t_replan = self.t_cur+self.replan_period
                self.info_space.reset_prediction()
                
            # retrive planned trajectory
            p_next, l_next, v_next, vl_next, p_plan, l_plan, v_plan, t_plan = self.planned_traj[t_next]

            # use the observation to verify if the planned waypoint is still safe
            if not self.safe_checker(p_plan, l_plan, v_plan, 0, self.t_cur, t_plan, self.obstacle_predictor, shadow_map, use_record=False, debug=False):
                print("Planned Traj is not safe", self.p_cur, self.v_cur, self.t_cur)
                self.planned_traj = None
                self.planned_node = None
                
                p_next, l_next, v_next, vl_next, safe = self.close_loop(self.p_cur, self.l_cur, self.v_cur, self.vl_cur,
                                                          self.t_cur, t_next, self.obstacle_predictor, shadow_map, use_record=False, debug=True)
                self.plan_open_loop = safe
                self.t_replan = t_next
        else:
            p_next, l_next, v_next, vl_next, safe = self.close_loop(self.p_cur, self.l_cur, self.v_cur, self.vl_cur, 
                                                          self.t_cur, t_next, self.obstacle_predictor, shadow_map, use_record=False, debug=True)           
            self.plan_open_loop = safe
            self.t_replan = t_next
        
        print("Move to (", p_next, l_next, v_next, vl_next, t_next,")")
        self.p_cur = p_next
        self.l_cur = l_next
        self.v_cur = v_next          
        self.vl_cur = vl_next
        self.t_cur = t_next
        
        self.move_ego()
    

    def tick_fake(self, sensor_data, sensor_pose, dp = 0.5, octomap = False):

        t_next = np.round(self.t_cur + self.dt_sim, 4)
        # Update the information space based on sensor observation
        if octomap:
            shadow_map, shadow_list = self.info_space.update_octomap(self.t_cur, sensor_data, sensor_pose)
        else:
            shadow_map, shadow_list = self.info_space.update(self.t_cur, sensor_data, sensor_pose)
                
        self.p_cur += dp
        self.l_cur
        self.v_cur          
        self.t_cur = t_next
        self.move_ego()

    def move_ego(self):
        spwan_point = self.lattice.lattice_2_global(self.p_cur, self.l_cur)
        self.client.ego.set_transform(spwan_point)
        self.t_hist.append(self.t_cur)
        self.p_hist.append(self.p_cur)
        self.l_hist.append(self.l_cur)
        self.v_hist.append(self.v_cur)

    def plot_hist(self):
        fig, (ax1, ax2) = plt.subplots(1, 2)
        ax1.plot(self.p_hist, self.v_hist)
        ax1.set_xlabel('P [m]')
        ax1.set_ylabel('V [m/s]')
        
        ax2.plot(self.p_hist, self.l_hist)
        ax2.set_xlabel('P [m]')
        ax1.set_xlabel('P [m]')
        
        plt.show()





        