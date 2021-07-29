import numpy as np
from planning import Route
from risk_analysis.shadow import InOutBound

class Lattice:
    def __init__(self, route: Route, dp, dl, dv, dt, V_range, t_range):
        self.route = route
        self.dp = dp
        self.dt = dt
        self.dl = dl
        self.dv = dv
        self.t_range = t_range
        self.V_range = V_range

        self.p_list = None
        self.stations = []
                

    def add_node(self, node):
        p = np.round(node.p,4)
        ifadd = self.stations[p].add_node(node)
        return ifadd

    def reset(self, p0):
        self.p_list = np.round(np.arange(p0, self.route.total_length, self.dp), 4)
        self.stations = {}

        
        for p in self.p_list:
            cur_route_section = self.route.get_route_section(p)
            l_min = cur_route_section.l_left[0]
            l_max = cur_route_section.l_right[-1]
            self.stations[p] = Station(p, l_min, l_max, self.dl, self.dv, self.dt, self.t_range, self.V_range)

    def lattice_2_global(self, p, l):
        # given progress and lateral position in the lattice, find the global coordinates
        
        # if (p,l) is a lattice
        p_round = np.round(p,4)
        l_round = np.round(l,4)
        if p_round in self.stations:
            cur_station = self.stations[p_round]
            if l_round in cur_station.vertices:
                cur_vertex = cur_station.vertices[l_round]
                if cur_vertex.global_transform is None:
                    cur_vertex.global_transform = self.route.find_global_position(p, l)
                return cur_vertex.global_transform
        
        return self.route.find_global_position(p, l) 

    def lattice_2_lane(self, p, l):
        # given progress and lateral position in the lattice, find the lanelet id and ds
        p_round = np.round(p,4)
        l_round = np.round(l,4)
        cur_station = self.stations[p_round]
        cur_vertex = cur_station.vertices[l_round]
        if cur_vertex.lane_info is None:
            cur_vertex.lane_info = self.route.find_nominal_lane(p)
        return cur_vertex.lane_info

    def lattice_exist(self, p, l):
        p_round = np.round(p,4)
        l_round = np.round(l,4)
        if p_round in self.stations:
            cur_station = self.stations[p_round]
            if l_round in cur_station.vertices:
                return True
        return False
         
class Station:
    def __init__(self, p, l_min, l_max, dl, dv, dt, t_range, V_range):
        self.p = p
        self.l_min = l_min
        self.l_max = l_max
        self.l_list = np.arange(np.ceil(self.l_min/dl), np.floor(self.l_max/dl)+1)*dl
        
        self.vertices = {}
        for l in self.l_list:
            self.vertices[l] = Vertex(p, l, dv, dt, t_range, V_range)
   
    def add_node(self, node):
        l = np.round(node.l)
        ifadd = self.vertices[l].add_node(node)
        return ifadd

class Vertex:
    def __init__(self, p, lateral_dis, dv, dt, t_range, V_range):
        self.p = p
        self.l = lateral_dis
        self.t_list = np.round(np.arange(t_range[0], t_range[1]+1, dt),4)
        self.v_list = np.round(np.arange(V_range[0], V_range[1], dv),4)
        self.n_t = self.t_list.shape[0]
        self.n_v = self.v_list.shape[0]
        self.node_list = (self.n_t*self.n_v)*[None]
        self.global_transform = None # in [x,y,z]
        self.lane_info = None  #tuple of (lanelet_id, ds)

    def get_idx(self, t, v):
        idx_t = np.searchsorted(self.t_list, t, side='right')-1
        idx_v = np.searchsorted(self.v_list, v, side='right')-1
        idx = idx_v*self.n_t+idx_t
        return idx_t, idx_v, idx
    
    def add_node(self, node):
        idx_t, idx_v, idx = self.get_idx(node.t,node.v)
        
        if self.node_list[idx] is None:
            self.node_list[idx] = node
            return True
        else:
            if node.cost<self.node_list[idx].cost:
                raise RuntimeWarning("lower cost")
                                
                #raise NameError('lower cost?')
                self.node_list[idx] = node
                return True
            else:
                # if node.v == 0:
                #     print("add fail", node.p, node.l, node.t, node.v, self.node_list[idx].p, self.node_list[idx].t, self.node_list[idx].v)
                return False
    

class Node:
    def __init__(self, p, l, t, v, parent, cost2come, cost2go):
        self.p = p
        self.l = l
        self.t = t
        self.v = v
        self.parent = parent
        self.cost2come = cost2come
        self.cost2go = cost2go
        self.cost = cost2come+cost2go
        self.predicted_shadows = None

    def add_predict_shadow(self, predicted_shadows):
        # for shadow in predicted_shadows:
        #     if shadow.root.id == 135 and isinstance(shadow.root, InOutBound):
        #         print(self.t, shadow.root.ds_out, shadow.root.ds_in)
        self.predicted_shadows = predicted_shadows

    def __lt__(self, obj):
        """self < obj."""
        return self.cost < obj.cost

    def __le__(self, obj):
        """self <= obj."""
        return self.cost <= obj.cost

    def __eq__(self, obj):
        """self == obj."""
        return self.cost == obj.cost

    def __ne__(self, obj):
        """self != obj."""
        return self.cost != obj.cost

    def __gt__(self, obj):
        """self > obj."""
        return self.cost > obj.cost

    def __ge__(self, obj):
        """self >= obj."""
        return self.cost >= obj.cost

