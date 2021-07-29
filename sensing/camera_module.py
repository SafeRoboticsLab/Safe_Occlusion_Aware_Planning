#!/usr/bin/env python3

"""
The camera wraper for carla environment

"""

import sys
from datetime import datetime
import random
import numpy as np
import numpy.matlib as npm

import carla
from carla import ColorConverter as cc
from abc import abstractmethod
from sensing.sensor import SensorBase
import weakref
import cv2
import math

import matplotlib.pylab as p



class CameraBase(SensorBase):
    """
    Base class of camera
    """
    def __init__(self, world, blueprint_library, vehicle, filter, width, height, fov,
            fps,  x, y, z, roll, pitch, yaw, attach):
        # properties of camera
        self.img_width = width
        self.img_height = height
        self.h_fov = fov # horiontal field of view
        self.fps = fps

        # blueprint for the camera
        self._bp = blueprint_library.find(filter)

        # set up camera parameters
        if self.h_fov>0:
            self._bp.set_attribute('fov', str(self.h_fov))
        self._bp.set_attribute('image_size_x', str(self.img_width))
        self._bp.set_attribute('image_size_y', str(self.img_height))
        self._bp.set_attribute('sensor_tick', str(1.0/fps))

        self.T_v2s = carla.Transform(carla.Location(x=x, y=y, z=z),
                            carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)) # sensor frame w.r.t vehicle frame
        self._actor = world.spawn_actor(self._bp, self.T_v2s, 
                            attach_to=vehicle, attachment_type=attach)

        # images and related variables that will be updated at each callback
        self.image = None
        self.frame = None
        self.timestamp = None
        self.trans = None

        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self._actor.listen(lambda data: self.__callback__(weak_self, data))

        self.vis_name = None

    def destroy(self):
        self._actor.stop()
        self._actor.destroy()
        self._actor = None

    def setup_visualizer(self, name = 'camera'):
        
        self.vis_name = name
        cv2.namedWindow(self.vis_name, cv2.WINDOW_NORMAL)
        
    
    def update_visualizer(self):
        if self.vis_name is not None and (self.image is not None):
            screen = self.vis_name#cv2.cvtColor(self.vis_name, cv2.COLOR_RGB2BGR)
            cv2.imshow(screen, self.image)
            cv2.waitKey(1)
    
    def set_intrinsic(self,K):
        self._actor.calibration = K


    @staticmethod
    @abstractmethod
    def __callback__(weak_self, data):
        pass


class Camera(CameraBase):
    """
    class of RGB camera
    """
    def __init__(self, world, blueprint_library, vehicle, width=1920, height=1080, 
            fov=110, fps=30, x=-0.5, y=0, z=1.8, roll=0, pitch=0, yaw=0, attach=carla.AttachmentType.Rigid):

        filter = 'sensor.camera.rgb'
        super().__init__(world, blueprint_library, vehicle, filter, width, height, fov, 
                fps, x, y, z, roll, pitch, yaw, attach)

    @staticmethod
    def __callback__(weak_self, data):
        self = weak_self()
        if not self:
            return
        data.convert(cc.Raw)
        self.frame = data.frame
        self.timestamp = data.timestamp
        self.trans = data.transform

        # convert to uint8 BGR images
        array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (data.height, data.width, 4))
        array = array[:, :, :3]
        self.image = array#[:, :, ::-1]




class DepthCamera(CameraBase):
    """
    class of Depth camera
    """
    def __init__(self, world, blueprint_library, vehicle, width=1920, height=1080, 
            fov=110, fps=30, x=-0.5, y=0, z=1.8, roll=0, pitch=0, yaw=0, attach=carla.AttachmentType.Rigid):

        filter = 'sensor.camera.depth'
        super().__init__(world, blueprint_library, vehicle, filter, width, height, fov, 
                fps, x, y, z, roll, pitch, yaw, attach)

    @staticmethod
    def __callback__(weak_self, data):
        self = weak_self()
        if not self:
            return
        data.convert(cc.Raw)
        self.frame = data.frame
        self.timestamp = data.timestamp
        self.trans = data.transform

        # convert to uint8 BGR images
        raw_data = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        raw_data = np.reshape(raw_data, (data.height, data.width, 4))
        raw_data = raw_data[:, :, :3].astype(np.float64)
        depth = (raw_data[:,:,2]+raw_data[:,:,1]*256+raw_data[:,:,0]*256*256)/(256*256*256-1)*1000
        self.image = depth # depth in meter
        
    def update_visualizer(self):
        if self.vis_name is not None and (self.image is not None):
            screen = self.vis_name#cv2.cvtColor(self.vis_name, cv2.COLOR_RGB2BGR)
            img2show = cv2.normalize(np.log(self.image), None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8UC1)
            cv2.imshow(screen, img2show)
            cv2.waitKey(10)
    
    def get_point_cloud(self, max_depth=900):
        # (Intrinsic) K Matrix
        if self.image is None:
            return None
        k = np.identity(3)
        k[0, 2] = self.img_width / 2.0
        k[1, 2] = self.img_height / 2.0
        k[0, 0] = k[1, 1] = self.img_width / \
            (2.0 * math.tan(self.h_fov * math.pi / 360.0))

        # 2d pixel coordinates
        pixel_length = self.img_width * self.img_height
        u_coord = npm.repmat(np.r_[self.img_width-1:-1:-1],
                        self.img_height, 1).reshape(pixel_length)
        v_coord = npm.repmat(np.c_[self.img_height-1:-1:-1],
                        1, self.img_width).reshape(pixel_length)

        depth = np.reshape(self.image, pixel_length)

        max_depth_indexes = np.where(depth > max_depth)
        depth = np.delete(depth, max_depth_indexes)
        u_coord = np.delete(u_coord, max_depth_indexes)
        v_coord = np.delete(v_coord, max_depth_indexes)

        # pd2 = [u,v,1]
        p2d = np.array([u_coord, v_coord, np.ones_like(u_coord)])

        # P = [X,Y,Z]
        p3d = np.dot(np.linalg.inv(k), p2d)
        p3d *= depth
        p3d = p3d[[2,0,1],:]
        p3d[1,:] = -p3d[1,:]
        
        return p3d

class SemanticCamera(CameraBase):
    """
    class of Seamntic camera
    """
    def __init__(self, world, blueprint_library, vehicle, width=1920, height=1080, 
            fov=110, fps=30, x=-0.5, y=0, z=1.8, roll=0, pitch=0, yaw=0, attach=carla.AttachmentType.Rigid):

        filter = 'sensor.camera.semantic_segmentation'
        super().__init__(world, blueprint_library, vehicle, filter, width, height, fov, 
                fps, x, y, z, roll, pitch, yaw, attach)
        self.label = None

    @staticmethod
    def __callback__(weak_self, data):
        self = weak_self()
        if not self:
            return
        # first, try to get label
        data.convert(cc.Raw)
        label = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        label = np.reshape(label, (data.height, data.width, 4))
        self.label = label[:,:,2]

        # convert to City scapes palette
        data.convert(cc.CityScapesPalette)
        self.frame = data.frame
        self.timestamp = data.timestamp
        self.trans = data.transform

        # convert to uint8 BGR images
        array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (data.height, data.width, 4))
        array = array[:, :, :3]
        self.image = array#[:, :, ::-1]



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

        depth = DepthCamera(world, blueprint_library, vehicle, fps=10)
        semantic = SemanticCamera(world, blueprint_library, vehicle)

        depth.setup_visualizer("Depth")
        semantic.setup_visualizer("Semantic")

        dt0 = datetime.now()
        

        while True:
            try:
                depth.update_visualizer()
                semantic.update_visualizer()
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
        depth.destroy()
        semantic.destroy()
        