#!/usr/bin/env python3

"""
The GNSS wraper for carla environment

"""

import weakref
import math
import carla
from sensing.sensor import SensorBase


class IMU(SensorBase):
    def __init__(self, world, blueprint_library, vehicle):
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        self._bp = blueprint_library.find('sensor.other.imu')
        self._actor = world.spawn_actor(
            self._bp, carla.Transform(), attach_to=vehicle)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self._actor.listen(
            lambda sensor_data: self.__callback__(weak_self, sensor_data))

    @staticmethod
    def __callback__(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)
    
    def destroy(self):
        self._actor.stop()
        self._actor.destroy()
        self._actor = None