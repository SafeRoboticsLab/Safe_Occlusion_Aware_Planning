#!/usr/bin/env python3

"""
The GNSS wraper for carla environment

"""

import weakref
from sensing.sensor import SensorBase
import carla


class LaneInvasionSensor(SensorBase):
    def __init__(self, world, blueprint_library, vehicle, hud=None):
        self.hud = hud
        self._bp = blueprint_library.find('sensor.other.lane_invasion')
        self._actor = world.spawn_actor(self._bp, carla.Transform(), attach_to=vehicle)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self._actor.listen(lambda event: self._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        if self.hud is not None:
            self.hud.notification('Crossed line %s' % ' and '.join(text))

    def destroy(self):
        self._actor.stop()
        self._actor.destroy()
        self._actor = None