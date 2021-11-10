import copy
from risk_analysis.shadow import InOutBound
from risk_analysis import ISpace
from planning import Lattice, A_star
from client import SynchronousClient
import numpy as np
from matplotlib import pyplot as plt


class HybridGame():
    def __init__(self, client: SynchronousClient, octomap: str, ego_path: list,
                    v_max = 20, a_max = 8, sensor_range = 100, dp = 0.5, dl = 0.5, dt_sim = 0.05, dt_sense=0.1, dt_plan = 0.5, dv = 0.5, T_plan = 5, T_replan = 0.5):

        self.client = client
        
        self.dt_sim = dt_sim
        self.dt_plan = dt_plan
        self.dt_sense = dt_sense

        self.replan_period = T_replan

        self.info_space = ISpace(self.client, octomap, ego_path, 
                            range = sensor_range, ds = 0.5, v_max=v_max, a_max=a_max)
        self.lattice = Lattice(self.info_space.route, dp = dp, dl = dl, dv = dv,
                                dt = dt_plan, V_range=[0, v_max], t_range = [0, T_plan])
        self.open_loop = A_star(self.info_space, self.lattice, v_max = v_max, a_max_accel= a_max/2, a_max_decel=a_max, dt = dt_plan, T_plan = T_plan)
        self.close_loop = self.info_space.state_checker.plan_close_loop
        self.safe_checker = self.info_space.state_checker.is_safe_plan
        self.obstacle_predictor = self.info_space.object_manager.predict_obstacle
        
        self.planned_traj = None

        self.t_cur = 0
        self.t_replan = None
        self.step_cur = 0

        self.p_cur = None
        self.l_cur = None
        self.v_cur = None
        self.vl_cur = None

        self.p_0 = None
        self.goal_p = None
        self.goal_t = None

        self.plan_open_loop = True 
        
        self.t_hist = []
        self.p_hist = []
        self.l_hist = []
        self.v_hist = []
        self.dangerzone_list = []

        self.debug = False
        

    def initialize_game(self, p0, l0, v0, goal_p = None, goal_t=None):

        self.lattice.reset(p0)
        self.t_replan = 0
        self.t_cur = 0
        self.step_cur = -1
                
        self.p_cur = p0
        self.l_cur = l0
        self.vl_cur = 0
        self.v_cur = v0
        self.p_0 = p0
        if goal_p:
            self.goal_t = goal_t
        if goal_t:
            self.goal_p = p0+goal_p
        self.move_ego()
       
    def tick(self, sensor_data, sensor_pose, baseline = True):

        # do a perception update
        shadow_map, shadow_list = self.info_space.update(self.t_cur, sensor_data, sensor_pose)
        """ Generate Current Danger Zone"""
        _, _ , _ =  self.safe_checker(self.p_cur, self.l_cur, self.v_cur, self.vl_cur, self.t_cur, self.t_cur, 
                                self.obstacle_predictor, shadow_map, record_danger_zone=True, use_record=False, debug=False)
        # if not safe:
        #     raise RuntimeError("Enter Unsafe region")
    
        self.dangerzone_list = self.info_space.state_checker.dangerzone_list
        
        # reaching the end of game 
        if self.goal_t is not None and self.t_cur>=self.goal_t:
            return False

        # next time step
        t_next = np.round((self.step_cur+1)*self.dt_sim, 4)
        
        remain = self.step_cur%2
        if remain != 0:
            # intermediate step, no collision checking
            if t_next not in self.planned_traj:
                return False
            p_next, l_next, v_next, vl_next, _, _, _, _, _ = self.planned_traj[t_next]
            self.p_cur = p_next
            self.l_cur = l_next
            self.v_cur = v_next          
            self.vl_cur = vl_next
            self.t_cur = t_next
            
        else:
            if self.plan_open_loop:
                # if replan is needed
                if self.t_cur >= self.t_replan:
                    if self.goal_p is not None and self.p_cur>=self.goal_p:
                        pass
                    else:
                        
                        self.lattice.reset(self.p_cur)
                        if baseline:
                            print("replanning baseline")
                            self.open_loop.plan_openloop(self.p_cur, self.l_cur, self.v_cur, self.t_cur, self.goal_p, shadow_map, self.debug)
                        else:
                            print("replanning close loop")
                            self.open_loop.plan(self.p_cur, self.l_cur, self.v_cur, self.t_cur, self.goal_p, shadow_map, shadow_list, self.debug)
                        self.planned_traj = self.open_loop.get_plan(self.dt_sim)
                        self.t_replan = self.t_cur+self.replan_period
                        print("next replan at ", self.t_replan)
                        self.info_space.reset_prediction()
                        if self.planned_traj is None:
                            print("no open loop solution, try close loop")
                            self.tick_close_loop(t_next, shadow_map)
                            #self.debug = True

                
                if t_next not in self.planned_traj:
                    return False
                # retrive planned trajectory        
                p_next, l_next, v_next, vl_next, p_plan, l_plan, v_plan, vl_plan, t_plan = self.planned_traj[t_next]

                # use the observation to verify if the planned waypoint is still safe
                safe, _ , _ =  self.safe_checker(p_plan, l_plan, v_plan, vl_plan, self.t_cur, t_plan, 
                                        self.obstacle_predictor, shadow_map, use_record=False, debug=False)
                if safe:
                    self.p_cur = p_next
                    self.l_cur = l_next
                    self.v_cur = v_next          
                    self.vl_cur = vl_next
                    self.t_cur = t_next
                else:
                    print("Planned Traj is not safe", self.p_cur, self.v_cur, self.t_cur, p_plan, l_plan, v_plan, t_plan)
                    self.tick_close_loop(t_next, shadow_map)
            else:
                self.tick_close_loop(t_next, shadow_map)
        
        self.move_ego()
        return True

    def tick_close_loop(self, t_next, shadow_map):
        t_next_sense = self.t_cur+self.dt_sense
        print("run close loop")
    
        p_next_sense, l_next_sense, v_next_sense, vl_next_sense, safe = self.close_loop(self.p_cur, self.l_cur, self.v_cur, self.vl_cur,
                                                        self.t_cur, t_next_sense, self.obstacle_predictor, shadow_map, use_record=False, debug=False)
     
        self.plan_open_loop = safe
        self.planned_traj = None
        self.t_replan = t_next
        self.subsample_traj(p_next_sense, l_next_sense, v_next_sense, vl_next_sense, t_next_sense)
        p_next, l_next, v_next, vl_next, _, _, _, _,_ = self.planned_traj[t_next]

        self.p_cur = p_next
        self.l_cur = l_next
        self.v_cur = v_next          
        self.vl_cur = vl_next
        self.t_cur = t_next

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
        print("Move to (", self.p_cur, self.l_cur, self.v_cur, self.vl_cur, self.t_cur,")")
        spwan_point = self.lattice.lattice_2_global(self.p_cur, self.l_cur)
        self.client.ego.set_transform(spwan_point)
        self.t_hist.append(self.t_cur)
        self.p_hist.append(self.p_cur)
        self.l_hist.append(self.l_cur)
        self.v_hist.append(self.v_cur)
        self.step_cur += 1

    def plot_hist(self):
        fig, (ax1, ax2) = plt.subplots(1, 2)
        ax1.plot(self.p_hist, self.v_hist)
        ax1.set_xlabel('P [m]')
        ax1.set_ylabel('V [m/s]')
        
        ax2.plot(self.p_hist, self.l_hist)
        ax2.set_xlabel('P [m]')
        ax1.set_xlabel('P [m]')
        
        plt.show()

    def subsample_traj(self, p_next_sense, l_next_sense, v_next_sense, vl_next_sense, t_next_sense):
        # print("subsample close loop traj from ", self.t_cur, "to", t_next_sense)
        # print("From", self.p_cur, self.l_cur, self.v_cur, self.vl_cur)
        # print("to ", p_next_sense, l_next_sense, v_next_sense, vl_next_sense)
        dt = t_next_sense-self.t_cur
        t_sampled = np.round(np.flip(np.arange(dt, 0, -self.dt_sim)), 4)

        ## longitudinal trajectory
        a_lon = (v_next_sense - self.v_cur)/dt

        v_sampled = np.round(self.v_cur+t_sampled*a_lon,4)
        p_sampled = np.round(self.p_cur+self.v_cur*t_sampled + 0.5*a_lon*t_sampled**2,4)

        ## lateral trajectory
        a_lat = (vl_next_sense -self.vl_cur)/dt

        l_sampled = np.ones_like(t_sampled)
        vl_sampled = np.ones_like(t_sampled)

        vl_sampled = np.round(a_lat*t_sampled + self.vl_cur,4)
        l_sampled = self.l_cur+0.5*a_lat*t_sampled**2+self.vl_cur*t_sampled
        
        t_sampled += self.t_cur
        t_sampled = np.round(t_sampled,3)

        self.t_replan = t_next_sense
        planned_traj = {}
        for t, p, l, v, vl in zip(t_sampled, p_sampled, l_sampled, v_sampled, vl_sampled):
            planned_traj[t] = (p, l, v, vl, p_next_sense, l_next_sense, v_next_sense, vl_next_sense, t_next_sense)  
        self.planned_traj = planned_traj
        

        


        