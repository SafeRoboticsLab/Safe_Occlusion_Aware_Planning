#!/usr/bin/env python3

"""
The Lidar wraper for carla environment

use Open3D for Lidar visuialization 
"""

import sys
import time
from datetime import datetime
import random
import numpy as np
from matplotlib import cm
import open3d as o3d
import weakref
import carla
from sensing.sensor import SensorBase


"""
parameteres for visualization 
"""

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (0, 0, 0), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses



class Lidar(SensorBase):
    def __init__(self, world, blueprint_library, vehicle, semantic=True, fps=10.0, upper_fov=15.0, lower_fov=-25.0,
            horizontal_fov = 180.0, channels=32.0, range=200, num_pnts=600000, noise=False, x=-0.5, y=0, z=1.8):
               
        # if ture, take semantic point cloud, otherwise take raw point cloud
        self.semantic = semantic
        
        # properties of lidars
        self.fps = fps 
        self.upper_fov = upper_fov
        self.lower_fov = lower_fov
        self.channels = channels
        self.range = range
        self.num_pnts = num_pnts #lidar's points per second
        self.horizontal_fov = horizontal_fov

        # blueprint for the lidar
        if self.semantic:
            self._bp = blueprint_library.find('sensor.lidar.ray_cast_semantic')
        else:
            self._bp = blueprint_library.find('sensor.lidar.ray_cast')
            if not noise:
                self._bp.set_attribute('dropoff_general_rate', '0.0')
                self._bp.set_attribute('dropoff_intensity_limit', '1.0')
                self._bp.set_attribute('dropoff_zero_intensity', '0.0')
            else:
                self._bp.set_attribute('noise_stddev', '0.2')

        # set lidar properties to the blueprint
        self._bp.set_attribute('upper_fov', str(self.upper_fov))
        self._bp.set_attribute('lower_fov', str(self.lower_fov))
        self._bp.set_attribute('channels', str(self.channels))
        self._bp.set_attribute('range', str(self.range))
        self._bp.set_attribute('rotation_frequency', str(self.fps))
        self._bp.set_attribute('points_per_second', str(self.num_pnts))
        self._bp.set_attribute('points_per_second', str(self.num_pnts))
        self._bp.set_attribute('horizontal_fov', str(self.horizontal_fov))
        # attach lidar to the vehicle
        self.T_v2s = carla.Transform(carla.Location(x, y, z)) # sensor frame w.r.t vehicle frame
        self._actor = world.spawn_actor(self._bp, self.T_v2s, attach_to=vehicle)

        # point clouds and related varibales that will be update at each callback
        self.semantic_raw_data = None
        self.point_cloud = None
        self.frame = None
        self.timestamp = None
        self.trans = None
        self.horizontal_angle = None
        

        # point cloud for visualization
        self.vis_points = o3d.geometry.PointCloud()

        # We need to pass the lambda a weak reference to self to avoid
        # circular reference.
        weak_self = weakref.ref(self)

        # set up call back function
        if self.semantic:
            self._actor.listen(lambda data: self.__semantic_lidar_callback__(weak_self, data))
        else:
            self._actor.listen(lambda data: self.__lidar_callback__(weak_self, data))

        self.visualizer = None


    @staticmethod           
    def __lidar_callback__(weak_self, raw_pointcloud):
        self = weak_self()
        if not self:
            return
        self.frame = raw_pointcloud.frame
        self.timestamp = raw_pointcloud.timestamp
        self.trans = raw_pointcloud.transform
        self.horizontal_angle = raw_pointcloud.horizontal_angle

        # convert the raw data
        data = np.copy(np.frombuffer(raw_pointcloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))
        intensity = data[:, -1]
        # Isolate the 3D data
        self.point_cloud = data[:, :-1]

        # We're negating the y to correclty visualize a world that matches
        # what we see in Unreal since Open3D uses a right-handed coordinate system
        
        self.point_cloud[:, :1] = -self.point_cloud[:, :1]

        # compute point color based on intensity
        intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
        int_color = np.c_[
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]
        self.vis_points.points = o3d.utility.Vector3dVector(self.point_cloud)
        self.vis_points.colors = o3d.utility.Vector3dVector(int_color)
        
    @staticmethod
    def __semantic_lidar_callback__(weak_self, raw_pointcloud):

        self = weak_self()
        if not self:
            return

        self.frame = raw_pointcloud.frame
        self.timestamp = raw_pointcloud.timestamp
        self.trans = raw_pointcloud.transform
        self.horizontal_angle = raw_pointcloud.horizontal_angle

        # convert the raw data
        data = np.frombuffer(raw_pointcloud.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))
        self.semantic_raw_data = data

        #data = data[data['ObjIdx']==0]

        # We're negating the y to correclty visualize a world that matches
        # what we see in Unreal since Open3D uses a right-handed coordinate system
        self.point_cloud = np.array([data['x'], -data['y'], data['z']]).T

        # # An example of adding some noise to our data if needed:
        # points += np.random.uniform(-0.05, 0.05, size=points.shape)

        # Colorize the pointcloud based on the CityScapes color palette
        labels = np.array(data['ObjTag'])
        int_color = LABEL_COLORS[labels]
        self.vis_points.points = o3d.utility.Vector3dVector(self.point_cloud)
        self.vis_points.colors = o3d.utility.Vector3dVector(int_color)
               
    def setup_visualizer(self, name = 'Lidar', width=960, height=540):
        """ function to setup open3d Visualizer for the lidar data"""

        self.frame_count = 0
        self.visualizer = o3d.visualization.Visualizer()

        self.visualizer.create_window(
            window_name=name,
            width=width,
            height=height,
            left=int(width/2),
            top=int(height/2))
        self.visualizer.get_render_option().background_color = [0.05, 0.05, 0.05]
        self.visualizer.get_render_option().point_size = 1
        self.visualizer.get_render_option().show_coordinate_frame = True

    def update_visualizer(self):
        """update the lidar visualizer"""
        if self.visualizer:
            if self.frame_count == 2:
                self.visualizer.add_geometry(self.vis_points)
            self.visualizer.update_geometry(self.vis_points)
            self.visualizer.poll_events()
            self.visualizer.update_renderer()
            time.sleep(0.001)
            self.frame_count += 1

    def destroy(self):
        self._actor.stop()
        self._actor.destroy()
        self._actor = None
        if self.visualizer:
            self.visualizer.destroy_window()
     


if __name__ == "__main__":
    # test if the code works
    host = 'localhost'
    port = 2000
    
    client = carla.Client(host, port)
    client.set_timeout(2.0)
    #world = client.reload_world()
    world = client.load_world('Town01')
    original_settings = world.get_settings()

    try:
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)

        settings.fixed_delta_seconds = 0.1
        settings.synchronous_mode = True
        settings.no_rendering_mode = False
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter("model3")[0]
        vehicle_transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        vehicle.set_autopilot(True)

        lidar = Lidar(world, blueprint_library, vehicle, semantic=False)

        lidar.setup_visualizer()

        dt0 = datetime.now()
        

        while True:
            try:
                lidar.update_visualizer()
                world.tick()
                process_time = datetime.now() - dt0
                sys.stdout.write('\r FPS: ' + str(1.0 / process_time.total_seconds()))
                sys.stdout.flush()
                dt0 = datetime.now()
            except KeyboardInterrupt:
                print("\n quitting!")
                break
            
    finally: 
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)
        vehicle.destroy()
        lidar.destroy()
        
        
        