#!/usr/bin/env python3


import numpy as np


import carla
#from mapping.road_network import RoadNetwork

import matplotlib
matplotlib.use('TKAgg')#conflict with opencv due to Qt issue
import matplotlib.pyplot as plt 

from client import SynchronousClient, PraseArgs

# we use this https://github.com/tsaoyu/PyVisiLibity package to predict visibility region
import visilibity as vis
import time


"""
PlaneMap represent the object in the world by assume the environment is 2D and all objects can be represented as polygons, 
"""


"""
    carla semantic labels
    #0 None
    #1 Building
    #2 Fences
    #3 Other
    #4 Pedestrian
    #5 Pole
    #6 RoadLines
    #7 Road
    #8 Sidewalk
    #9 Vegetation
    #10 Vehicle
    #11 Wall
    #12 TrafficSign
    #13 Sky
    #14 Ground
    #15 Bridge
    #16 RailTrack
    #17 GuardRail
    #18 TrafficLight
    #19 Static
    #20 Dynamic
    #21 Water
    #22 Terrain

"""

class BboxUtils():
    """
    utility functions to handle carla bounding boxes
    """



    @staticmethod
    def get_3d_bb_points(bbox):
        """
        Returns 3D bounding box 
        """

        cords = np.zeros((8, 4))
        extent = bbox.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])

        T_world = BboxUtils.get_matrix(carla.Transform(bbox.location, bbox.rotation))
        world_cords = np.dot(T_world, np.transpose(cords))[:3, :]
        return world_cords


    @staticmethod
    def get_2d_bb_points(bbox):
        """
        Returns 3D bounding box 
        """

        cords = np.zeros((4, 4))
        extent = bbox.extent
        # arrange in clockwise to satisify the requirement in PyVisiLibity
        cords[0, :] = np.array([extent.x, extent.y, 0, 1])
        cords[1, :] = np.array([extent.x, -extent.y, 0, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, 0, 1])
        cords[3, :] = np.array([-extent.x, extent.y, 0, 1])

        T_world = BboxUtils.get_matrix(carla.Transform(bbox.location, bbox.rotation))
        world_cords = np.dot(T_world, np.transpose(cords))[:2, :]
        return world_cords


    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


class PlaneMap():
    """
        A class defines the map in XY plane. 
        It stores bounding boxes for building, wall, and vehicles that may results occlusion
    """
    def __init__(self, client, bbox_filters = [1,11]):

        #self.road_network = RoadNetwork(client)

        # parameters of maps
        waypoints = client.map.generate_waypoints(2)
        self.max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + 50
        self.max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + 50
        self.min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - 50
        self.min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - 50

        #print(" xlim: [", self.min_x, self.max_x, "]. ylim: [", self.min_y, self.max_y, "]")

        self.static_bbox_list = []
        self.static_bbox_ctr_list = np.empty((0,2))
        self.static_bbox_vertex_list = []
        bbox_list_temp = []
        for filter in bbox_filters:
            bbox_list_temp.extend(client.world.get_level_bbs(filter))

        # filter bbox with hieght profile
        for bbox in bbox_list_temp:
            if (bbox.location.z - bbox.extent.z)<0.5 and (bbox.location.z + bbox.extent.z)>1.5:
                self.static_bbox_ctr_list = np.append(self.static_bbox_ctr_list, [[bbox.location.x, bbox.location.y]], axis = 0)
                self.static_bbox_vertex_list.append(BboxUtils.get_2d_bb_points(bbox))
    
        """ first attempt construct the environment at one step"""
        self.env_whole = None
        self.polygon_list = None
        self._construct_env()
    
    def _construct_env(self):
        # Define the points which will be the outer boundary of the environment
        # Must be COUNTER-CLOCK-WISE(ccw)
        t = time.time()

        p1 = vis.Point(self.max_x, self.min_y)
        p2 = vis.Point(self.max_x, self.max_y)
        p3 = vis.Point(self.min_x, self.max_y)
        p4 = vis.Point(self.min_x, self.min_y)
        # Load the values of the outer boundary polygon in order to draw it later
        walls = vis.Polygon([p1, p2, p3, p4])

        self.polygon_list = [walls]

        for bbox in self.static_bbox_vertex_list:
            # bbox is 2xN matrix
            p_list = []
            for i in range(bbox.shape[1]):
                p_list.append(vis.Point(bbox[0,i], bbox[1,i]))
            self.polygon_list.append(vis.Polygon(p_list))
        
        self.env_whole = vis.Environment(self.polygon_list)
        elapsed = time.time() - t
        #print("Time required for construction: ", elapsed)

    def gen_FOV(self, observer_location):
        t = time.time()

        epsilon = 0.0000001
        observer = vis.Point(observer_location[0], observer_location[1])
        # Necesary to generate the visibility polygon
        observer.snap_to_boundary_of(self.env_whole, epsilon)
        observer.snap_to_vertices_of(self.env_whole, epsilon)
        vis_polygon = vis.Visibility_Polygon(observer, self.env_whole, epsilon)
        elapsed = time.time() - t
        #print("Time required for construction: ", elapsed)

        

        return vis_polygon

    @staticmethod
    def polygon_to_vertex(polygon):
        vertex_x = []
        vertex_y = []
        for i in range(polygon.n()):
            x = polygon[i].x()
            y = polygon[i].y()
            
            vertex_x.append(x)
            vertex_y.append(y)
        return (vertex_x, vertex_y)
                                                             
    
    def plot_visibility(self, vis_polygon):
        # plot polygon first
        plt.figure()
        # ignore the wall
        for bbox in self.static_bbox_vertex_list:
            # bbox is 2xN matrix
            x = []
            y = []
            for i in range(bbox.shape[1]):
                x.append(bbox[0,i])
                y.append(bbox[1,i])
            x.append(bbox[0,0])
            y.append(bbox[1,0])
            plt.plot(x, y, 'red')
        
        vis_vertex = self.polygon_to_vertex(vis_polygon)
        vertex_x = vis_vertex[0]
        vertex_x.append(vertex_x[0])
        vertex_y = vis_vertex[1]
        vertex_y.append(vertex_y[0])
        plt.plot(vertex_x, vertex_y, 'blue')
        plt.show()



if __name__ == "__main__":
    args = PraseArgs()
    client = None
    try:
        # set up carla client
        client = SynchronousClient(args)       
        client.setup_ego_vehicle()
        # client.spawn_random_vehicles(20)
        client.add_lidar()

        map = PlaneMap(client)
        for _ in range(4):
            client.tick()
            client.render()

        ego_location = client.get_ego_transform().location

        vis_poly = map.gen_FOV([ego_location.x, ego_location.y])
        map.plot_visibility(vis_poly)


        
        # # set up map viewer
        # frame = Frame(args.width, args.height)
        # hud = HUD(args.width, args.height)
        # map_view = MapViewer(args)
        # map_view.start(client, obstacle_list=map.static_bbox_vertex_list, follow_ego=False)
        # clock = pygame.time.Clock()
        # while True:
        #     clock.tick_busy_loop(60)
        #     client.tick()
        #     hud.tick(client)
        #     map_view.tick(client)

        #     #time.sleep(0.01)
        #     client.render()
        #     map_view.render(frame)
        #     hud.render(frame)
        #     pygame.display.flip()


    finally:
        if client:
            client.destroy()



        





        



    

    


