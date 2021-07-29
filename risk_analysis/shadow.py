
import sys
import numpy as np
from recordclass import dataobject
from pympler.asizeof import asizeof

class Shadow():
    '''
       A shadow is represneted as a tree 
    
    '''
    # reduce memeory
    __slots__ = 'risk_bound', 'root', 'parent_id', 'leaf_list', 'leaf_depth'
    def __init__(self):        
        
        
        # Which type bound is shadow is the root of the treeBoolean
        #     - True for in bound
        #     - False for out bound
    
        self.risk_bound = True

        # root of the tree
        #     - InBound if risk_bound is True
        #     - OutBound if risk_bound is False
        
        self.root = None 

        #if the this is a backward shadow, this is the id of ego lane that treat this shadow as risky 
        self.parent_id = None 

        # If the leaf of the tree is one of following bound, it pointer and depth is append in this list
        # if the leaf is ShadowNode, which means it is too long, and search stopped. The node is not in this list
        self.leaf_list = []
        self.leaf_depth = []

    def __lt__(self, shadow2compare):
        """self < obj."""
        my_depth_max = 0
        for depth in self.leaf_depth:
            if depth is None:
                my_depth_max = np.inf
                break
            elif depth > my_depth_max:
                my_depth_max = depth

        other_depth_max = 0
        for depth in shadow2compare.leaf_depth:
            if depth is None:
                other_depth_max = np.inf
                break
            elif depth > other_depth_max:
                other_depth_max = depth
        return my_depth_max < other_depth_max
        
class ShadowNode():
    # reduce memeory
    __slots__ = 'id', 'child_node', 'parent_node'
    def __init__(self, id):
        self.id = id
        self.child_node = None # None only takes 16 bytes, but [] takes 64 bytes
        self.parent_node = None

class InBound():
    __slots__ = 'id', 'ds_in', 't_in', 'ds_in_hist', 't_in_hist', 'child_node', 'parent_node'
    def __init__(self, id, ds, t):
        # id of the lane where the inbound is located
        self.id = id
        # ds of the lane where the inbound is located
        self.ds_in = ds
        # time step when inbound is observed
        self.t_in = t

        # default to None to save some memory, but make the code hard to read
        # List of 1D np.array
        # each np.array record the relative s between historical inbounds and current in bound
        self.ds_in_hist = None 
        # List of 1D np.array
        # each np.array record the time step historical inbounds are observed
        self.t_in_hist = None

        self.child_node = None

        self.parent_node = None


class OutBound():
    __slots__ = 'id', 'ds_out', 't_out', 'child_node', 'parent_node'
    def __init__(self, id, ds, t):
        # id of the lane where the inbound is located
        self.id = id
        # ds of the lane where the inbound is located
        self.ds_out = ds
        # time step when inbound is observed
        self.t_out = t

        self.child_node = None

        self.parent_node = None



class InOutBound():
    __slots__ = 'id', 'ds_in', 'ds_out', 't_in', 't_out', 'ds_in_hist', 't_in_hist', 'child_node', 'parent_node'

    def __init__(self, id, ds_in, ds_out, t_in, t_out):
        self.id = id
        self.ds_in = ds_in
        self.ds_out = ds_out
        self.t_in = t_in
        self.t_out = t_out
        self.ds_in_hist = None
        self.t_in_hist = None
        self.child_node = None # always None. Add this to avoid nasty if statement in update
        self.parent_node = None # always None. Add this to avoid nasty if statement in update





