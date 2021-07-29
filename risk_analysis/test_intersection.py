from risk_analysis.reachable_set import double_integrator_reachable_tube
import numpy as np
import matplotlib.pyplot as plt


def time2finish(v0, a, dis, v_max):

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




    
    


    


v0 = 15
a_max = 5
v_max = 20

d_intersection0 = 18
d_intersection1 = d_intersection0+9
d_intersection2 = d_intersection1+9

t_slow_enter, v_slow_enter = time2finish(v0, -a_max, d_intersection0, v_max)
if not np.isnan(t_slow_enter):
    t_slow_exit = time2finish(v_slow_enter, a_max, d_intersection1-d_intersection0, v_max)[0]+t_slow_enter
    t_slow_exit2 = time2finish(v_slow_enter, a_max, d_intersection2-d_intersection0, v_max)[0]+t_slow_enter
    print(t_slow_enter, t_slow_exit)

    BRT_slow = double_integrator_reachable_tube(0, v_max, t_slow_enter, t_slow_exit, a_max, 0, 20, 5+2,BRS=True)
    plt.plot(*BRT_slow.exterior.xy, color='red', label='Slow')

    # BRT_slow2 = double_integrator_reachable_tube(0, v_max, t_slow_exit, t_slow_exit2, a_max, 0, 20, 5+2,BRS=True)
    # plt.plot(*BRT_slow2.exterior.xy, '--', color='red', label='Slow interseciton2')


t_fast_enter, v_fast_enter = time2finish(v0, a_max, d_intersection0, v_max)
t_fast_exit = time2finish(v_fast_enter, a_max, d_intersection1-d_intersection0, v_max)[0]+t_fast_enter
t_fast_exit2 = time2finish(v_fast_enter, a_max, d_intersection2-d_intersection0, v_max)[0]+t_fast_enter

print(t_fast_enter, t_fast_exit)


BRT_fast = double_integrator_reachable_tube(0, v_max, t_fast_enter, t_fast_exit, a_max, 0, 20, 5+2,BRS=True)
plt.plot(*BRT_fast.exterior.xy, color='blue', label='Fast')
# BRT_fast2 = double_integrator_reachable_tube(0, v_max, t_fast_exit, t_fast_exit2, a_max, 0, 20, 5+2,BRS=True)
# plt.plot(*BRT_fast2.exterior.xy, '--', color='blue', label='Fast intersection2')
plt.legend()
plt.show()


intersection = BRT_fast.intersection(BRT_slow)
plt.plot(*intersection.exterior.xy)
plt.show()





