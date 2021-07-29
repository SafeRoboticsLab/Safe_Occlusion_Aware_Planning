import numpy as np
from matplotlib import pyplot as plt
from numpy.core.fromnumeric import searchsorted
from pyclothoids import Clothoid, SolveG2


class Trajectory:
    def __init__(self, road, sl_start, sl_end, xyt_start=None, xyt_end=None):
       
        self.sl_start = sl_start
        self.sl_end = sl_end
        
        
        if xyt_start is None:
            x, y, theta, kappa = road.route_to_global(sl_start[0], sl_start[1])
            self.xyt_start = [x, y, theta, kappa]
        else:
            self.xyt_start = xyt_start
        
        if xyt_end is None:
            x, y, theta, kappa = road.route_to_global(sl_end[0], sl_end[1])
            self.xyt_end = [x, y, theta, kappa]
        else:
            self.xyt_end = xyt_end
        
        self.path_set = SolveG2(self.xyt_start[0], self.xyt_start[1], self.xyt_start[2], self.xyt_start[3],
                        self.xyt_end[0], self.xyt_end[1], self.xyt_end[2], self.xyt_end[3])
        self.arc_length = sum([path.length for path in self.path_set])

    def create_traj(self, ego, v_start, t_start, dt):
        self.t_sart = t_start
        self.t_end = t_start+dt
        self.v_start = v_start
        

        # acceleration and velocity profile
        # first assume constant acceleration
        a = (self.arc_length-v_start*dt)/(0.5*dt**2)
        v_end = v_start + a*dt
        if v_end<=ego.v_max:
            self.a_profile = [[0, a]]
            self.v_end = v_end
        else:
            t_accel = (ego.v_max*dt-self.arc_length)/(ego.v_max/2.0-v_start/2.0)
            a = (ego.v_max-v_start)/t_accel
            self.a_profile = [[0,a], [t_accel, 0]]
            self.v_end = ego.v_max
        return self.v_end

    # def create_traj(self, v_start, v_end, t_start, dt):
    #     self.t_sart = t_start
    #     self.t_end = t_start+dt
    #     self.v_start = v_start
    #     self.v_end = v_end

    #     if self.arc_length > dt * v_end:
    #         return False

    #     t_accel = (v_end*dt-self.arc_length)/(v_end/2.0-v_start/2.0)
    #     a = (v_end-v_start)/t_accel
    #     if a>ego.a_max:
    #         return False
    #     else:
    #         self.a_profile = [[0,a], [t_accel, 0]]
    #         return True
        

    
    # def gen_waypoints(self, N, include_start=False, include_end=True):
    #     if not include_start:
    #         N += 1 
        
    #     if include_end:
    #         N += 1

    #     arc_s = np.linspace(0, self.arc_length, N, endpoint=include_end)
    #     t_global = np.linspace(self.t_sart, self.t_end, N, endpoint=include_end)
    #     if not include_start:
    #         arc_s = arc_s[1:]
    #         t_global = t_global[1:]
        
    #     waypoints = np.zeros((arc_s.shape[0], 3))
    #     path_idx = 0
    #     prev_sec_length = 0
    #     for i, s in enumerate(arc_s):
    #         while s>(prev_sec_length+self.path_set[path_idx].length):
    #             prev_sec_length += self.path_set[path_idx].length
    #             path_idx += 1
    #         sec_s = s-prev_sec_length
    #         x = self.path_set[path_idx].X(sec_s)
    #         y = self.path_set[path_idx].Y(sec_s)
    #         theta = self.path_set[path_idx].Theta(sec_s)
    #         waypoints[i,:] = [x, y, theta]
    #     return t_global, arc_s, waypoints



    def gen_waypoints(self, dt, include_start=False, include_end=True):
        if include_start:
            t_0 = 0
        else:
            t_0 = dt

        if include_end:
            t_1 = self.t_end-self.t_sart+dt
        else:
            t_1 = self.t_end-self.t_sart

        t = np.arange(t_0, t_1, dt)
        if len(self.a_profile)==1:
            arc_s = self.v_start*t+0.5*self.a_profile[0][1]*t**2
            v_list = self.v_start+self.a_profile[0][1]*t
        else:
            idx_v_max = searchsorted(t, self.a_profile[1][0])
            arc_s = 0*t
            v_list = 0*t
            arc_s[0:idx_v_max] = self.v_start*t[0:idx_v_max]+0.5*self.a_profile[0][1]*t[0:idx_v_max]**2
            v_list[0:idx_v_max] = self.v_start+self.a_profile[0][1]*t[0:idx_v_max]
            arc_s[idx_v_max:] = self.v_start*self.a_profile[1][0]+0.5*self.a_profile[0][1]*self.a_profile[1][0]**2+(t[idx_v_max:]-self.a_profile[1][0])*self.v_end
            v_list[idx_v_max:] = self.v_end
        if arc_s[-1] > self.arc_length:
            if arc_s[-1] -self.arc_length>1e-3:
                print("something wrong")
            arc_s[-1] = self.arc_length
        waypoints = np.zeros((t.shape[0], 3))
        path_idx = 0
        prev_sec_length = 0
        for i, s in enumerate(arc_s):
            while s>(prev_sec_length+self.path_set[path_idx].length):
                prev_sec_length += self.path_set[path_idx].length
                path_idx += 1
            sec_s = s-prev_sec_length
            x = self.path_set[path_idx].X(sec_s)
            y = self.path_set[path_idx].Y(sec_s)
            theta = self.path_set[path_idx].Theta(sec_s)
            waypoints[i,:] = [x, y, theta]
        t_global = t+self.t_sart

        l_list = np.linspace(self.sl_start[1], self.sl_end[1], (arc_s.shape[0]+1*(not include_start)), endpoint=include_end)
        s_list = np.linspace(self.sl_start[0], self.sl_end[0], (arc_s.shape[0]+1*(not include_start)), endpoint=include_end)
        if not include_start:
            l_list = l_list[1:]
            s_list = s_list[1:]
        return s_list, l_list, v_list, t_global, waypoints





            


         











