#!/usr/bin/env python3

"""
The Collision sensor wraper for carla environment

"""

import weakref
import math
import collections
from sensing.sensor import SensorBase
import carla


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


class CollisionSensor(SensorBase):
    def __init__(self, world, blueprint_library, vehicle, hud=None):
        self.history = []
        self.hud = hud
        self._bp = blueprint_library.find('sensor.other.collision')
        self._actor = world.spawn_actor(self._bp, carla.Transform(), attach_to=vehicle)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self._actor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        if self.hud is not None:
            self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)

    def destroy(self):
        self._actor.stop()
        self._actor.destroy()
        self._actor = None