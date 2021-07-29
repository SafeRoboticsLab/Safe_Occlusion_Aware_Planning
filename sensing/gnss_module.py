#!/usr/bin/env python3

"""
The GNSS wraper for carla environment

"""

import weakref
import carla
from sensing.sensor import SensorBase

class GNSS(SensorBase):
    """ Class for GNSS sensors"""

    def __init__(self, world, blueprint_library, vehicle):
        """Constructor method"""
        self.lat = 0.0
        self.lon = 0.0
        self._bp = blueprint_library.find('sensor.other.gnss')
        self._actor = world.spawn_actor(self._bp, carla.Transform(carla.Location(x=1.0, z=2.8)),
                                        attach_to=vehicle)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self._actor.listen(lambda event: self.__callback__(weak_self, event))

    @staticmethod
    def __callback__(weak_self, event):
        """GNSS method"""
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude

    def destroy(self):
        self._actor.stop()
        self._actor.destroy()
        self._actor = None
