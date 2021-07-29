import numpy as np
a_max = 6
def max_accel(v_ego, v_obs, a_max_obs, s_remain, plan_step):
        if a_max_obs == 0:
            s_obs_stop = s_remain
        else:
            s_obs_stop = v_obs**2/(2*a_max_obs) + s_remain
        
        print(s_obs_stop)

        a = plan_step**2/(2*a_max)
        b = v_ego*plan_step/a_max+0.5*plan_step**2
        c = v_ego**2/(2*a_max)+v_ego*plan_step-s_obs_stop

        
        omega = np.sqrt(b**2-4*a*c)
        a_sol = (-b+omega)/(2*a)
        v1 = v_ego+a_sol*plan_step
        p1 = v_ego*plan_step + 0.5*a_sol*plan_step**2
        stop = v1**2/(2*a_max)

        print(v1, p1, p1+stop, s_obs_stop)
        return (-b+omega)/(2*a)


print(max_accel(19, 8.5, 4, 23.521, 0.1))