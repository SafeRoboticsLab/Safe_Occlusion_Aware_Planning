#!/usr/bin/env python3

"""
Base class for all sensor wraper of Carla
"""


from abc import ABC, abstractmethod


class SensorBase(ABC):
    """
    Base class of sensor
    """
    def __init__(self):
        super().__init__()
        self._bp = None
        self._actor = None

    def get_pose(self):
        return self._actor.get_transform()

    @abstractmethod
    def destroy(self):
        pass
    