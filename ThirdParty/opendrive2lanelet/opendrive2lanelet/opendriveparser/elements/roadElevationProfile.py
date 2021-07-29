# -*- coding: utf-8 -*-

from opendrive2lanelet.opendriveparser.elements.road_record import RoadRecord
import numpy as np

__author__ = "Benjamin Orthen, Stefan Urban"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.2.0"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class ElevationProfile:
    """The elevation profile record contains a series of elevation records
    which define the characteristics of
    the road's elevation along the reference line.

    (Section 5.3.5 of OpenDRIVE 1.4)
    """

    def __init__(self):
        self.elevations = []
        self.start_pos_list = None

    def initialize(self):
        self.start_pos_list = [record[0].start_pos for record in self.elevations]



    def calc_z(self, sPos):
        if self.start_pos_list is None:
            self.initialize()
        if sPos.size>1:
            reverse = sPos[0]>sPos[1]
        else:
            reverse = False
        if reverse:
            sPos_left = np.flip(sPos)
        else:
            sPos_left = sPos

        ds_list = []
        for start_pos in reversed(self.start_pos_list):
            idx = np.searchsorted(sPos_left, start_pos)
            ds_list.append(sPos_left[idx:] - start_pos)
            sPos_left = sPos_left[:idx]

        z = np.array([])
        for i, ds in enumerate(reversed(ds_list)):
            
            coeff = self.elevations[i][0].polynomial_coefficients
            #z = a+b*ds+c*ds^2+d*ds^3
            z_local = coeff[0]+coeff[1]*ds+coeff[2]*ds**2+coeff[3]*ds**3
            z = np.hstack((z,z_local))
        
        if reverse:
            z = np.flip(z)
        
        return z



        


class ElevationRecord(RoadRecord):
    """The elevation record defines an elevation entry at a given reference line position.

    (Section 5.3.5.1 of OpenDRIVE 1.4)
    """
