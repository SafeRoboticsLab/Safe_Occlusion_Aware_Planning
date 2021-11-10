from matplotlib import pyplot as plt
from shapely.geometry import Polygon, LineString
from shapely.affinity import affine_transform
import numpy as np
from datetime import datetime


def double_integrator_reachable_tube(v0_min, v0_max, delta_t_min, delta_t_max, a_max_accel, a_max_decel, v_min, v_max, x_extend=0, n=20, BRS = False, oppo = False):
    """
    Given the current velocity v0, velocity bound [v_min, v_max], acceleration bound a_max, calculate the FRS w.r.t to current 
    x_extend : is the half length of the vehicle

    """
    v_cb, s_cb = double_integrator_reachable_tube_cb(v0_min, v0_max, delta_t_min, delta_t_max, a_max_accel, a_max_decel, v_min, v_max, n)
    v_fb, s_fb = double_integrator_reachable_tube_fb(v0_min, v0_max, delta_t_min, delta_t_max, a_max_accel, a_max_decel, v_min, v_max, n)
    
    s = np.hstack((s_cb-x_extend, s_fb+x_extend))
    v = np.hstack((v_cb, v_fb))

    shell = []
    for s_i, v_i in zip(s,v):
        if BRS and oppo:
            shell.append((s_i,-v_i))
        elif oppo:
            shell.append((-s_i,-v_i))
        elif BRS:
            shell.append((-s_i,v_i))
        else:
            shell.append((s_i,v_i))
    polygon = Polygon(shell)
    if polygon is None:
        print("is None", v0_min, v0_max, delta_t_min, delta_t_max, a_max_accel, a_max_decel, v_min, v_max, x_extend)
    return Polygon(shell) #, s_fb+x_extend, v_fb


def double_integrator_reachable_tube_cb(v0_min, v0_max, delta_t_min, delta_t_max, a_max_accel, a_max_decel, v_min, v_max, n=20):
    """
    calculate close bound by first decelerate then accelerate
    """
    if delta_t_min>0:
        r_v02max = (1-(v_max-v0_min)/delta_t_min/a_max_accel)/2
        r_min2max = 1 - (v_max - v_min)/a_max_accel/delta_t_min
        r = np.linspace(max(max(r_v02max, r_min2max), 0), 1, n, endpoint=True)
    else:
        r = np.linspace(0, 1, n, endpoint=True)

    v_cb_1 = v0_min - a_max_decel*r*delta_t_min
    s_cb_1 = v0_min*r*delta_t_min -0.5*a_max_decel*(r*delta_t_min)**2

    idx_stop = v_cb_1<v_min
    s_cb_1[idx_stop] = s_cb_1[idx_stop]+(v_cb_1[idx_stop]-v_min)**2/(2*a_max_decel)
    v_cb_1[idx_stop] = 0

    v_cb = v_cb_1+a_max_accel*(1-r)*delta_t_min
    s_cb = s_cb_1 + v_cb_1*(1-r)*delta_t_min + 0.5*a_max_accel*((1-r)*delta_t_min)**2

    idx_max = v_cb>v_max
    s_cb[idx_max] = s_cb[idx_max]-(v_cb[idx_max]-v_max)**2/(2*a_max_accel)
    v_cb[idx_max] = v_max

    if delta_t_max != delta_t_min:
        # bound for any delta that the object accelerate from v0_min from all time
        delta_t_sat = min(delta_t_max, (v_max - v0_max)/a_max_accel)
        if delta_t_sat > delta_t_min:
            delta_t_sample = np.linspace(delta_t_sat, delta_t_min, num=n, endpoint=True)
            v_bound = v0_max+a_max_accel*delta_t_sample
            v_bound[v_bound>v_max] = v_max
            s_bound = np.vstack((v_max*delta_t_sample-0.5*a_max_decel*delta_t_sample**2,
                            v0_max*delta_t_sample+0.5*a_max_accel*delta_t_sample**2))
            s_bound = np.min(s_bound, axis=0)

            v_cb = np.hstack((v_bound, v_cb))
            s_cb = np.hstack((s_bound, s_cb))
        
        delta_t_sat = min((v0_min-v_min)/a_max_decel, delta_t_max)
        if delta_t_sat > delta_t_min:
            # bound for any delta that the object decelerate from v0_min from all time
            delta_t_sample = np.linspace(delta_t_min, min((v0_min-v_min)/a_max_decel, delta_t_max), num=n, endpoint=True)
            v_bound = v0_min-a_max_decel*delta_t_sample
            v_bound[v_bound<v_min] = v_min
            s_bound = np.vstack((v_min*delta_t_sample + 0.5*a_max_accel*delta_t_sample**2,
                            v0_min*delta_t_sample-0.5*a_max_decel*delta_t_sample**2))
            s_bound = np.max(s_bound, axis=0)
            v_cb = np.hstack((v_cb, v_bound))
            s_cb = np.hstack((s_cb, s_bound))

    if v0_max != v0_min and v_cb[0]<v_max:
       v_cb = np.hstack((min(v0_max+a_max_accel*delta_t_min, v_max), v_cb))
       s_cb_max = v_max*delta_t_min-0.5*a_max_accel*delta_t_min**2
       s_cb = np.hstack((min(v0_max*delta_t_min+0.5*a_max_accel*delta_t_min**2, s_cb_max), s_cb))
    return v_cb, s_cb


def double_integrator_reachable_tube_fb(v0_min, v0_max, delta_t_min, delta_t_max, a_max_accel, a_max_decel, v_min, v_max, n=20):
    """
    calculate further bound by first accelerate then decelerate
    """
    if delta_t_max>0:
        r_v02min = (1+(v_min-v0_max)/delta_t_max/a_max_decel)/2
        r_max2min = 1 - (v_max - v_min)/a_max_accel/delta_t_max
        r = np.linspace(max(max(r_v02min, r_max2min), 0), 1, n, endpoint=True)
    else:
        r = np.linspace(0, 1, n, endpoint=True)

    v_fb_1 = v0_max + a_max_accel*r*delta_t_max
    s_fb_1 = v0_max*r*delta_t_max +0.5*a_max_accel*(r*delta_t_max)**2

    idx_max = v_fb_1>v_max
    s_fb_1[idx_max] = s_fb_1[idx_max]-(v_fb_1[idx_max]-v_max)**2/(2*a_max_accel)
    v_fb_1[idx_max] = v_max

    v_fb = v_fb_1-a_max_decel*(1-r)*delta_t_max
    s_fb = s_fb_1 + v_fb_1*(1-r)*delta_t_max - 0.5*a_max_decel*((1-r)*delta_t_max)**2

    idx_stop = v_fb<v_min
    s_fb[idx_stop] = s_fb[idx_stop]+(v_fb[idx_stop]-v_min)**2/(2*a_max_decel)
    v_fb[idx_stop] = 0

    if v0_max != v0_min and v_fb[0]>v_min:
        v_fb = np.hstack((max(v0_min-a_max_decel*delta_t_max, v_min), v_fb))
        s_fb_min = v_min*delta_t_max + 0.5*a_max_decel*delta_t_max**2
        s_fb = np.hstack((max(v0_min*delta_t_max-0.5*a_max_decel*delta_t_max**2, s_fb_min), s_fb))

    return v_fb, s_fb

    
           
    if v0_max != v0_min and v_fb[0]>v_min:
        v_fb = np.hstack((max(v0_min-a_max*delta_t_max, v_min), v_fb))
        s_fb_min = v_min*delta_t_max + 0.5*a_max*delta_t_max**2
        s_fb = np.hstack((max(v0_min*delta_t_max-0.5*a_max*delta_t_max**2, s_fb_min), s_fb))



def double_integrator_stop_distance(v_ego, a_ego, v_obj, a_obj, x_extend=0):
    s_stop = max(v_ego**2 /(2*a_ego) - v_obj**2/(2*a_obj), 0)+x_extend
    return s_stop


def double_integrator_stop_dangerzone(v_ego, a_max_decel, v_max, x_extend = 0, n=20):
    """
    If we can assume that stopping (without collision with object ahead) is an invariable safe state
    the danger zone can be represent as  area under the curve s=(v_ego^2 - v_obj^2)/(2*a)+x_extend

    """ 

    v_obj = np.linspace(0, v_ego, num=n)
    s_stop = (v_ego**2 - v_obj**2)/(2*a_max_decel)

    shell = []

    # check if collision already. In the velocity axis, should be -inf to inf. 
    # But we kind of assume other object will have velocity larger than 0 and less than v_max
    
    
    for s, v in zip(s_stop, v_obj):
        shell.append((s+x_extend, v))

    if x_extend > 0:
        shell.extend([(x_extend, v_max), (-x_extend, v_max), (-x_extend, 0)])

    return Polygon(shell), s_stop[0]+x_extend



def double_integrator_merge_dangerzone(p_shield, t_shield, a_max_accel, a_max_decel, v_max, x_extend=0, n=20):
    v_cb, s_cb = double_integrator_reachable_tube_cb(0, v_max, t_shield, t_shield, a_max_accel, a_max_decel, 0, v_max, n)
    v_fb, s_fb = double_integrator_reachable_tube_fb(0, v_max, t_shield, t_shield, a_max_accel, a_max_decel, 0, v_max, n)
    shell = [(-x_extend, -v_max), (-x_extend, v_max)]

    for s_i, v_i in zip(s_cb,v_cb):
        shell.append((-s_i+p_shield+x_extend, v_i))
    for s_i, v_i in zip(s_fb,v_fb):
        shell.append((s_i+p_shield+x_extend, -v_i))

    s_same_max = -np.min(s_i)+p_shield+x_extend
    s_oppo_max = np.max(s_fb)+p_shield+x_extend
    
    return Polygon(shell), s_same_max, s_oppo_max


def translate_polygon(original: Polygon, s_trans):
    return affine_transform(original, [1,0,0,1,s_trans, 0])


if __name__ == "__main__":
    for i in range(0,11):
        danger1= double_integrator_reachable_tube(10+i, 10+i, 1, 1, 4, 8, 0, 20, 2, BRS = False)
        plt.plot(*danger1.exterior.xy,'--')

    danger1= double_integrator_reachable_tube(10, 20, 1, 1, 4, 8, 0, 20, 2, BRS = False)
    #danger2= double_integrator_reachable_tube(10, 10, 1, 1, 4, 8, 0, 20, 2, BRS = True)
    plt.plot(*danger1.exterior.xy,'-')
    #plt.plot(*danger2.exterior.xy)

    plt.show()








