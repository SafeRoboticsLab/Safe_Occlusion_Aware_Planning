#!/usr/bin/env python3

import glooey
import imgviz
import numpy as np
import pyglet
import trimesh
import trimesh.transformations as tf
import trimesh.viewer

from mapping.road_network import RoadNetwork
from mapping.plane_map import PlaneMap
import time



import octomap
from os import path
import carla


""" 
3D static occupancy map using Octomap
"""

class OccupancyMap():
    def __init__(self, client=None,  filename = None, resolution = 0.1):
        if client is not None:
            self.road_network = RoadNetwork(client)
            self.plane_map = PlaneMap(client)
        else:
            self.road_network = None
            self.plane_map = None

        # store the occupancy information of the environment
        if filename is None:
            self.resolution = resolution
            self.octree = octomap.OcTree(self.resolution)
        else:
            print("Loading ", filename)
            if path.isfile(filename):
                # read from file
                if filename[-2:] == "ot": # full tree
                    self.octree = octomap.OcTree.read(str.encode(filename))
                    self.resolution = self.octree.getResolution()
                elif filename[-2:] == "bt": # binary tree
                    self.octree = octomap.OcTree.readBinary(str.encode(filename))
                    self.resolution = self.octree.getResolution()
                else:
                    raise Exception("Octomap are saved to either \".ot\" or \".bt\" files")
                print("Import "+filename+" with resolution "+str(self.resolution))
            else:
                self.resolution = resolution
                self.octree = octomap.OcTree(self.resolution)

    def insert_point_cloud(self, pc, sensor_pose, range = -1):
        '''
            Given carla point cloud and Carla.Transform of the sensor, 
            insert to the octree
        '''
        # first tranform point cloud in world frame
        if pc is not None:
            if pc.shape[0]!=3:
                pc = pc.T
            
            N_pts = pc.shape[1]
            if isinstance(sensor_pose, list):
                T_world, origin = self.get_matrix_list(sensor_pose)
            else:
                T_world, origin = self.get_matrix(sensor_pose)
            
            pc_world = np.dot(T_world, np.append(pc, np.ones((1,N_pts)), axis=0))[:3, :].T
            pc_world = np.asarray(pc_world)
            self.octree.insertPointCloud(pc_world, origin, maxrange = range, lazy_eval = False)

    def visualize(self):
        """ 
        visualize the octomap with pyglet
        """
        self.octree.updateInnerOccupancy()
        print("Start Octomap Visualization")

        # define parameters
        data = imgviz.data.arc2017()
        camera_info = data['camera_info']
        K = np.array(camera_info['K']).reshape(3, 3)
        width=camera_info['width']
        height=camera_info['height']

        # get free and occupied grid
        occupied, _ = self.octree.extractPointCloud()
        #frontier = self.gen_frontier()
        
        print("load point cloud")
        window = pyglet.window.Window(
            width=int(1280), height=int(960)
        )

        @window.event
        def on_key_press(symbol, modifiers):
            if modifiers == 0:
                if symbol == pyglet.window.key.Q:
                    window.on_close()

        gui = glooey.Gui(window)
        hbox = glooey.HBox()
        hbox.set_padding(5)

        camera = trimesh.scene.Camera(
            resolution=(width, height), focal=(K[0, 0], K[1, 1])
        )

        # initial camera pose
        camera_transform = np.array(
            [
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, -5],
                [0.0, 0.0, 0.0, 1.0],
            ],
        )

 

        occupied_geom = trimesh.voxel.ops.multibox(
            occupied, pitch=self.resolution, colors=[0.0, 0.0, 0.0, 0.5]
        )

        # frontier_geom = trimesh.voxel.ops.multibox(
        #     frontier, pitch=self.resolution, colors=[1.0, 0, 0, 0.5]
        # )
        scene = trimesh.Scene(camera=camera, geometry=[occupied_geom])#, frontier_geom])
        scene.camera_transform = camera_transform
        hbox.add(self.labeled_scene_widget(scene, label='octomap'))


        gui.add(hbox)
        pyglet.app.run()
     
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

        origin = np.array([location.x, location.y, location.z])
        return matrix, origin

    @staticmethod
    def get_matrix_list(transform):
        """
        Creates matrix from carla transform.
        """
        c_y = np.cos(np.radians(transform[5]))
        s_y = np.sin(np.radians(transform[5]))
        c_r = np.cos(np.radians(transform[3]))
        s_r = np.sin(np.radians(transform[3]))
        c_p = np.cos(np.radians(transform[4]))
        s_p = np.sin(np.radians(transform[4]))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = transform[0]
        matrix[1, 3] = transform[1]
        matrix[2, 3] = transform[2]
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        origin = np.array(transform[:3])
        return matrix, origin 

    def write_to_file(self, filename):
        """
        Write file header and complete tree to a ot file (serialization)
        """
        self.octree.write(str.encode(filename))
        print("Save octomap to "+filename)

    def write_to_binary_file(self, filename):
        """
        Writes OcTree to a binary file using writeBinary().
        The OcTree is first converted to the maximum likelihood estimate and pruned.
        """

        self.octree.writeBinary(str.encode(filename))

    @staticmethod
    def labeled_scene_widget(scene, label):
        vbox = glooey.VBox()
        vbox.add(glooey.Label(text=label, color=(255, 255, 255)), size=0)
        vbox.add(trimesh.viewer.SceneWidget(scene))
        return vbox

    def check_visibility(self, sensor_loc, end_loc, debug=False):
        # dt0 =  time.time()
        if isinstance(sensor_loc, list):
            origin = np.array([sensor_loc[0], sensor_loc[1], sensor_loc[2]],dtype=np.double)
        else:
            origin = np.array([sensor_loc.x, sensor_loc.y, sensor_loc.z],dtype=np.double)
        
           

        if isinstance(end_loc, carla.Location):
            end = np.array([end_loc.x, end_loc.y, end_loc.z],dtype=np.double)
        else:
            end = np.array([end_loc[0], end_loc[1], end_loc[2]], dtype=np.double)
        
        direction = end-origin
        dis = np.linalg.norm(direction)
        #direction = np.array([0,0,1], dtype=np.double)

        end_hit = np.array([0,0,0], dtype = np.double)

        hit = self.octree.castRay(origin, direction, end_hit)
        
        if debug:
            print("sensor at", origin, "look to ", end, "hit", end_hit)

        hit_dis = np.linalg.norm(end_hit - origin)
        if hit and hit_dis - dis<-0.5:
            #print("target at ", end, "with direction", direction, " hit at ", end_hit)
            return False
        return True

    def gen_frontier(self, sensor_pose = None, V_fov = None):
        
        frontier = []

        for it in self.octree.begin_leafs():
            is_frontier = False
            if not self.octree.isNodeOccupied(it):
                x_cur = it.getX()
                y_cur = it.getY()
                z_cur = it.getZ()

                # if sensor_pose is not None and V_fov is not None:
                #     pt_sensor = 
                v_angle = np.arctan2(z_cur, (x_cur**2+y_cur**2)**0.5)

                # if v_angle > np.deg2rad(5):
                #     continue
                
                def check_buffer():
                    num_free = 0

                    buff = np.array([-self.resolution, 0, self.resolution])
                    for x_buff in x_cur+buff:
                        for y_buff in y_cur+buff:
                            for z_buff in z_cur+buff: 
                                node_ptr = self.octree.search([x_buff, y_buff, z_buff])
                                if node_ptr.is_Null():
                                    #num_free +=1
                                    origin = np.array([0,0,0],dtype=np.double)
                                    direction = np.array([x_buff, y_buff, z_buff],dtype=np.double)
                                    end_hit = np.array([0,0,0], dtype = np.double)
                                    hit = self.octree.castRay(origin, direction, end_hit)
                                    if hit and np.linalg.norm(end_hit)>2:
                                        num_free +=1
                    if num_free>=5:
                        return True
                    else:
                        return False
                            
                           
                
                is_frontier = check_buffer()
                if is_frontier:
                    center = np.array([x_cur, y_cur, z_cur])

                    dimension = max(1, round(it.getSize() / self.resolution))
                    origin = center - (dimension / 2 - 0.5) * self.resolution
                    indices = np.column_stack(np.nonzero(np.ones((dimension, dimension, dimension))))
                    points = origin + indices * np.array(self.resolution)
                    frontier.append(points)
        if len(frontier) == 0:
            print("No frontier are found")
            frontier_arr = np.zeros((0, 3), dtype=float)
        else:
            frontier_arr = np.concatenate(frontier, axis=0)
            print("found ",len(frontier), " frontiers")
            

        return frontier_arr
                    





        

