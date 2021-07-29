#!/usr/bin/env python3

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import sys
import random
import time
import re
import numpy as np


import carla
from carla import VehicleLightState as vls

from sensing import Lidar, Camera, DepthCamera, SemanticCamera, GNSS, IMU, CollisionSensor, LaneInvasionSensor, Radar
from client.prase_args import PraseArgs
from client.change_weather import set_weather



'''
This class define the carla world in the client side
'''

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]





# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================
  


class SynchronousClient(object):
    def __init__(self, args, dt = 0.1):

        print("Initializing Carla Client")

        # parameters
        self.args = args
        self.recording_enabled = False
        self.recording_start = 0
        self._weather_presets = find_weather_presets()
        self._weather_index = 0

        # sensors list
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.lidar_list = []
        self.radar_sensor_list = []
        self.depth_camera_list = []
        self.rgb_camera_list = []
        self.spectator_camera = None

        # initialize the client
        host = self.args.host
        port = self.args.port

        self.client = carla.Client(host, port)
        self.client.set_timeout(20.0)
        self.world = self.client.load_world(args.map)

        self.actor_role_name = args.rolename
        self.original_settings = self.world.get_settings()

        self.ego = None # ego vehilce
        self.vehicle_list = []
        # apply the setting of Synchronous Mode
        self.world.set_weather(carla.WeatherParameters.ClearNoon)
        #set_weather(self.world, self.args)
        settings = self.world.get_settings()
        self.traffic_manager = self.client.get_trafficmanager(8000)
        self.traffic_manager.set_synchronous_mode(True)
        settings.fixed_delta_seconds = dt
        settings.synchronous_mode = True
        settings.no_rendering_mode = False
        self.world.apply_settings(settings)

        self.blueprint_library = self.world.get_blueprint_library()

        # get map
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

    def setup_ego_vehicle(self, filter = None, pose=None, autopilot = True, spectator_pose=[-4, 0, 2, 0, 2.0, 0]):
        """
        Initialize the ego vehicle and spectator camera 

        Input: 
            filter: str to filter the vehicle type from blueprint library
            pose: list [x, y, z, roll, pitch, yaw] or Calra.Transform
        """
        if filter is None:
            filter = self.args.filter
        ego_bp = random.choice(self.blueprint_library.filter(filter))
        ego_bp.set_attribute('role_name', self.args.rolename)
        if pose is None:
            spawn_point = random.choice(self.map.get_spawn_points())
        elif isinstance(pose, list):
            spawn_point = carla.Transform(
                        carla.Location(x=pose[0], y=pose[1], z=pose[2]),
                            carla.Rotation(roll=pose[3], pitch=pose[4], yaw=pose[5]))
        else:
            spawn_point = pose
            
        self.ego = self.world.spawn_actor(ego_bp, spawn_point)
        self.ego.set_autopilot(autopilot)

        # set up a spectator camera
        self.setup_ego_sensors()
        self.spectator_camera = Camera(self.world, self.blueprint_library, self.ego, 
                        width=self.args.width*2, height=self.args.height*2, 
                        fov=120, fps=1/self.args.dt, 
                        x=spectator_pose[0], y=spectator_pose[1], z=spectator_pose[2],
                        roll=spectator_pose[3], pitch=spectator_pose[4], yaw=spectator_pose[5],
                        attach=carla.AttachmentType.SpringArm)
        calibration = np.identity(3)
        calibration[0, 2] = self.args.width / 2.0
        calibration[1, 2] = self.args.height / 2.0
        calibration[0, 0] = calibration[1, 1] = self.args.width / (2.0 * np.tan(120 * np.pi / 360.0))
        self.spectator_camera.set_intrinsic(calibration)
        self.spectator_camera.setup_visualizer(name="Spectator")

        print("Set actor ", self.ego.id, " as the ego vehicle")
        
    def setup_ego_sensors(self):
        self.imu_sensor = IMU(self.world, self.blueprint_library, self.ego)
        self.gnss_sensor = GNSS(self.world, self.blueprint_library, self.ego)
        self.collision_sensor = CollisionSensor(self.world, self.blueprint_library, self.ego)
        self.lane_invasion_sensor = LaneInvasionSensor(self.world, self.blueprint_library, self.ego)
        pass
    
    def add_lidar(self, visualize = True, semantic=True, fps=10.0, upper_fov=15.0, lower_fov=-25.0, h_fov = 180,
            channels=32.0, range=200, num_pnts=600000):
        new_lidar = Lidar(self.world, self.blueprint_library, self.ego,
                        semantic = semantic, fps = fps, 
                        upper_fov=upper_fov, lower_fov = lower_fov, horizontal_fov=h_fov,
                        channels = channels, range = range, num_pnts = num_pnts)

        self.lidar_list.append(new_lidar)
        if visualize:
            self.lidar_list[-1].setup_visualizer(name = "Lidar"+str(len(self.lidar_list)))
        
    def add_depth_camera(self, visualize = True, width=1920, height=1080, fov=110, fps=10.0, x=-0.5, y=0, z=1.8, roll=0, pitch=0, yaw=0, name = None):
        
        self.depth_camera_list.append(DepthCamera(self.world, self.blueprint_library, self.ego, 
                width, height, fov, fps, x, y, z, roll, pitch, yaw))
        if visualize:
            if name is None:
                name = "Depth"+str(len(self.lidar_list))
            self.depth_camera_list[-1].setup_visualizer(name = name)
        
    def get_actors(self):
        """
        return list of actors in the world
        """
        return self.world.get_actors()

    def get_ego_transform(self):
        """
        return carla.Transform for the ego vehicle
        """
        if self.ego is not None:
            return self.ego.get_transform()
        else:
            return None

    def spawn_objects(self, filter, waypoint):
        ego_bp = random.choice(self.blueprint_library.filter(filter))
        spawn_point = waypoint.transform
        spawn_point.location.z += 0.3
        actor = self.world.spawn_actor(ego_bp, spawn_point)
        self.vehicle_list.append(actor.id)
        return actor

    def spawn_random_vehicles(self, number_of_vehicles = 50):
        random.seed(int(time.time()))

        self.traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        self.traffic_manager.set_hybrid_physics_mode(False)

        blueprints = self.blueprint_library.filter('vehicle.*')

        # if args.safe:
        #     blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        #     blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        #     blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        #     blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        #     blueprints = [x for x in blueprints if not x.id.endswith('t2')]

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = self.map.get_spawn_points()  # Returns a list of recommended spawning points for vehicles
        number_of_spawn_points = len(spawn_points)

        if number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif number_of_vehicles > number_of_spawn_points:
            number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            # prepare the light state of the cars to spawn
            # light_state = vls.NONE
            # if args.car_lights_on:
            light_state = vls.Position | vls.LowBeam | vls.LowBeam

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, self.traffic_manager.get_port()))
                .then(SetVehicleLightState(FutureActor, light_state)))

        for response in self.client.apply_batch_sync(batch, True):
            if response.error:
                print(response.error)
            else:
                self.vehicle_list.append(response.actor_id)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.world.set_weather(preset[0])

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = Radar(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def tick(self):
        self.world.tick()

    def render(self):
        """
        Update sensor images
        """
        if self.spectator_camera is not None:
            self.spectator_camera.update_visualizer()
        for lidar in self.lidar_list:
            lidar.update_visualizer()
        for depth in self.depth_camera_list:
            depth.update_visualizer()
        
    def destroy(self):
        if self.collision_sensor is not None:
            self.collision_sensor.destroy()
            self.collision_sensor = None

        if self.lane_invasion_sensor is not None:
            self.lane_invasion_sensor.destroy()
            self.lane_invasion_sensor = None

        if self.gnss_sensor is not None:
            self.gnss_sensor.destroy()
            self.gnss_sensor = None

        if self.imu_sensor is not None:
            self.imu_sensor.destroy()
            self.imu_sensor = None
        
        for radar in self.radar_sensor_list:
            radar.destroy()
        self.radar_sensor_list = []

        for camera in self.rgb_camera_list:
            camera.destroy()
        self.rgb_camera_list = []

        for camera in self.depth_camera_list:
            camera.destroy()
        self.depth_camera_list = []

        for lidar in self.lidar_list:
            lidar.destroy()
        self.lidar_list = []
        self.world.apply_settings(self.original_settings)




if __name__ == "__main__":
    args = PraseArgs()
    try:
        # set up carla client
        client = SynchronousClient(args)
        client.setup_ego_vehicle()
        client.spawn_random_vehicles(20)
        client.add_lidar(visualize=True)

        while True:
            client.tick()
            
            #time.sleep(0.01)
            client.render()
        
    finally:
        client.destroy()