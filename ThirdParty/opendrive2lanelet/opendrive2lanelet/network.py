# -*- coding: utf-8 -*-


"""Module to contain Network which can load an opendrive object and then export
to lanelets. Iternally, the road network is represented by ParametricLanes."""

from commonroad.scenario.scenario import Scenario
from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2lanelet.opendriveparser.elements.opendrive import OpenDrive

from opendrive2lanelet.utils import encode_road_section_lane_width_id
from opendrive2lanelet.conversion_lanelet_network import ConversionLaneletNetwork
from opendrive2lanelet.converter import OpenDriveConverter


__author__ = "Benjamin Orthen, Stefan Urban"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.2.0"
__maintainer__ = "Sebastian Maierhofer"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class Network:
    """Represents a network of parametric lanes, with a LinkIndex
    which stores the neighbor relations between the parametric lanes.

    Args:

    """

    def __init__(self):
        self._planes = []
        self._link_index = None
        self.opendrive = None

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def load_opendrive_str(self, opendrive_str: str, precision: float = 0.5):
        """Load all elements of an OpenDRIVE network to a parametric lane representation

        Args:
          opendrive:

        """
        self.opendrive = parse_opendrive(opendrive_str)

        self._link_index = LinkIndex()
        self._link_index.create_from_opendrive(self.opendrive)

        # Convert all parts of a road to parametric lanes (planes)
        for road in self.opendrive.roads:
            road.planView.precalculate(precision)

            # The reference border is the base line for the whole road
            reference_border = OpenDriveConverter.create_reference_border(
                road.planView, road.lanes.laneOffsets
            )

            # A lane section is the smallest part that can be converted at once
            for lane_section in road.lanes.lane_sections:

                parametric_lane_groups = OpenDriveConverter.lane_section_to_parametric_lanes(
                    lane_section, reference_border
                )

                self._planes.extend(parametric_lane_groups)
                # for parametric_lane in parametric_lane_groups:
                #     self._planes.append(parametric_lane)
                #     self._road_id_planes_idx[parametric_lane.id_] = len(self._planes)-1

    def load_opendrive(self, opendrive: OpenDrive, precision: float = 0.5):
        """Load all elements of an OpenDRIVE network to a parametric lane representation

        Args:
          opendrive:

        """

        if not isinstance(opendrive, OpenDrive):
            raise TypeError()

        self._link_index = LinkIndex()
        self._link_index.create_from_opendrive(opendrive)

        # Convert all parts of a road to parametric lanes (planes)
        for road in opendrive.roads:
            road.planView.precalculate(precision)

            # The reference border is the base line for the whole road
            reference_border = OpenDriveConverter.create_reference_border(
                road.planView, road.lanes.laneOffsets
            )

            # A lane section is the smallest part that can be converted at once
            for lane_section in road.lanes.lane_sections:

                parametric_lane_groups = OpenDriveConverter.lane_section_to_parametric_lanes(
                    lane_section, reference_border
                )

                self._planes.extend(parametric_lane_groups)
                # for parametric_lane in parametric_lane_groups:
                #     self._planes.append(parametric_lane)
                #     self._road_id_planes_idx[parametric_lane.id_] = len(self._planes)-1


               

    def export_lanelet_network(
        self, filter_types: list = None, concatenate = False, precision: float = 0.5
    ) -> "ConversionLaneletNetwork":
        """Export network as lanelet network.

        Args:
          filter_types: types of ParametricLane objects to be filtered. (Default value = None)

        Returns:
          The converted LaneletNetwork object.
        """
        # Convert groups to lanelets
        lanelet_network = ConversionLaneletNetwork()

        for parametric_lane in self._planes:
            if filter_types is not None and parametric_lane.type not in filter_types:
                continue

            lanelet = parametric_lane.to_lanelet(precision)

            lanelet.predecessor = self._link_index.get_predecessors(parametric_lane.id_)
            lanelet.successor = self._link_index.get_successors(parametric_lane.id_)
            if parametric_lane.id_ in self._link_index.merge_dict:
                lanelet.merge = self._link_index.merge_dict[parametric_lane.id_]
            if parametric_lane.id_ in self._link_index.split_dict:
                lanelet.split = self._link_index.split_dict[parametric_lane.id_]
            

            lanelet_network.add_lanelet(lanelet)

        # prune because some
        # successorIds get encoded with a non existing successorID
        # of the lane link
        lanelet_network.prune_network()

        if concatenate:
            lanelet_network.concatenate_possible_lanelets()
        
        # Correct the lane boundary for merge and split 
        lanelet_network.join_and_split_possible_lanes()
        # find conflict lines in the network
        lanelet_network.find_cross(self._link_index.junction_list)
        lanelet_network.convert_all_lanelet_ids()
        return lanelet_network

    def export_commonroad_scenario(
        self, dt: float = 0.1, benchmark_id=None, filter_types=None, concatenate = False
    ):
        """Export a full CommonRoad scenario

        Args:
          dt:  (Default value = 0.1)
          benchmark_id:  (Default value = None)
          filter_types:  (Default value = None)

        Returns:

        """

        scenario = Scenario(
            dt=dt, scenario_id = None, benchmark_id=benchmark_id if benchmark_id is not None else "none"
        )

        scenario.add_objects(
            self.export_lanelet_network(
                filter_types=filter_types
                if isinstance(filter_types, list)
                else ["driving", "onRamp", "offRamp", "exit", "entry"],
                concatenate=concatenate
            )
        )

        return scenario

class LinkIndex:
    """Overall index of all links in the file, save everything as successors, predecessors can be found via a reverse search"""

    def __init__(self):
        self._successors = {}
        self._predecessors = {}

        self.merge_dict = {}
        self.split_dict = {}
        self.junction_list = []

    def create_from_opendrive(self, opendrive):
        """Create a LinkIndex from an OpenDrive object.

        Args:
          opendrive: OpenDrive style object.

        Returns:

        """
        self._add_junctions(opendrive)
        # Extract link information from road lanes
        for road in opendrive.roads:
            for lane_section in road.lanes.lane_sections:
                for lane in lane_section.allLanes:
                    parametric_lane_id = encode_road_section_lane_width_id(
                        road.id, lane_section.idx, lane.id, -1
                    )

                    # Not the last lane section? > Next lane section in same road
                    if lane_section.idx < road.lanes.getLastLaneSectionIdx():
                        successorId = encode_road_section_lane_width_id(
                            road.id, lane_section.idx + 1, lane.link.successorId, -1
                        )

                        self.add_link(parametric_lane_id, successorId, lane.id >= 0)

                    # Last lane section! > Next road in first lane section
                    # Try to get next road
                    elif (
                        road.link.successor is not None
                        and road.link.successor.elementType != "junction"
                    ):

                        next_road = opendrive.getRoad(road.link.successor.element_id)

                        if next_road is not None:

                            if road.link.successor.contactPoint == "start":
                                successorId = encode_road_section_lane_width_id(
                                    next_road.id, 0, lane.link.successorId, -1
                                )

                            else:  # contact point = end
                                successorId = encode_road_section_lane_width_id(
                                    next_road.id,
                                    next_road.lanes.getLastLaneSectionIdx(),
                                    lane.link.successorId,
                                    -1,
                                )
                            self.add_link(parametric_lane_id, successorId, lane.id >= 0)

                    # Not first lane section? > Previous lane section in same road
                    if lane_section.idx > 0:
                        predecessorId = encode_road_section_lane_width_id(
                            road.id, lane_section.idx - 1, lane.link.predecessorId, -1
                        )

                        self.add_link(predecessorId, parametric_lane_id, lane.id >= 0)

                    # First lane section! > Previous road
                    # Try to get previous road
                    elif (
                        road.link.predecessor is not None
                        and road.link.predecessor.elementType != "junction"
                    ):

                        prevRoad = opendrive.getRoad(road.link.predecessor.element_id)

                        if prevRoad is not None:

                            if road.link.predecessor.contactPoint == "start":
                                predecessorId = encode_road_section_lane_width_id(
                                    prevRoad.id, 0, lane.link.predecessorId, -1
                                )

                            else:  # contact point = end
                                predecessorId = encode_road_section_lane_width_id(
                                    prevRoad.id,
                                    prevRoad.lanes.getLastLaneSectionIdx(),
                                    lane.link.predecessorId,
                                    -1,
                                )
                            self.add_link(
                                predecessorId, parametric_lane_id, lane.id >= 0
                            )

    def add_link(self, parametric_lane_id, successor, reverse: bool = False):
        """

        Args:
          parametric_lane_id:
          successor:
          reverse:  (Default value = False)

        Returns:

        """

        # if reverse, call function recursively with switched parameters
        if reverse:
            self.add_link(successor, parametric_lane_id)
            return

        if parametric_lane_id not in self._successors:
            self._successors[parametric_lane_id] = []
        
        if successor not in self._predecessors:
            self._predecessors[successor] = []

        if successor not in self._successors[parametric_lane_id]:
            self._successors[parametric_lane_id].append(successor)

            # if a lane section has more than one lanes successor, it is considered as split
            if len(self._successors[parametric_lane_id]) > 1:
                for split_id in self._successors[parametric_lane_id][:-1]:
                    self._add_split(successor, split_id)
        
        if parametric_lane_id not in self._predecessors[successor]:
            self._predecessors[successor].append(parametric_lane_id)

            # if a lane section has more than one lanes predecessor, it is considered as merge
            # and all predecessor are considered as conflicts
            if len(self._predecessors[successor]) > 1:
                for merge_id in self._predecessors[successor][:-1]:
                    self._add_merge(parametric_lane_id, merge_id)

    def _add_merge(self, merge_id_1, merge_id_2):
        """
        add merged lane parametric id to each's dict
        Args:
          merge_id_1:
          merge_id_2:

        Returns:

        """
        # init in the map
        if merge_id_1 not in self.merge_dict:
            self.merge_dict[merge_id_1] = []
        if merge_id_2 not in self.merge_dict:
            self.merge_dict[merge_id_2] = []
        
        # add id to each other's list
        if merge_id_1 not in self.merge_dict[merge_id_2]:
            self.merge_dict[merge_id_2].append(merge_id_1)

        if merge_id_2 not in self.merge_dict[merge_id_1]:
            self.merge_dict[merge_id_1].append(merge_id_2)
    
    def _add_split(self, split_id_1, split_id_2):
        """
        add splitted lane parametric id to each's dict
        Args:
          split_id_1:
          split_id_2:

        Returns:

        """
        # init in the map
        if split_id_1 not in self.split_dict:
            self.split_dict[split_id_1] = []
        if split_id_2 not in self.split_dict:
            self.split_dict[split_id_2] = []
        
        # add id to each other's list
        if split_id_1 not in self.split_dict[split_id_2]:
            self.split_dict[split_id_2].append(split_id_1)

        if split_id_2 not in self.split_dict[split_id_1]:
            self.split_dict[split_id_1].append(split_id_2)
    

    def _add_junctions(self, opendrive):
        """

        Args:
          opendrive:

        Returns:

        """
        # add junctions to link_index
        # if contact_point is start, and laneId from connecting_road is negative
        # the connecting_road is the successor
        # if contact_point is start, and laneId from connecting_road is positive
        # the connecting_road is the predecessor
        # for contact_point == end it's exactly the other way
        for junction in opendrive.junctions:
            cur_junction = []
            for connection in junction.connections:
                incoming_road = opendrive.getRoad(connection.incomingRoad)
                connecting_road = opendrive.getRoad(connection.connectingRoad)
                contact_point = connection.contactPoint 

                for lane_link in connection.laneLinks:
                    if contact_point == "start":
                        # first make sure that connecting road is successor
                        if lane_link.toId < 0:
                            # decide which lane section to use (first or last)
                            if lane_link.fromId < 0:
                                lane_section_idx = (
                                    incoming_road.lanes.getLastLaneSectionIdx()
                                )
                            else:
                                lane_section_idx = 0
                        else: # if the connecting road is preccessor
                            #print("Invert junction path order")
                            if lane_link.fromId > 0:
                                lane_section_idx = (
                                    incoming_road.lanes.getLastLaneSectionIdx()
                                )
                            else:
                                lane_section_idx = 0
                        incoming_road_id= encode_road_section_lane_width_id(
                            incoming_road.id, lane_section_idx, lane_link.fromId, -1
                        )
                        connecting_road_id = encode_road_section_lane_width_id(
                            connecting_road.id, 0, lane_link.toId, -1
                        )
                        self.add_link(
                                incoming_road_id, connecting_road_id, lane_link.toId > 0
                            )
                    else:
                        if lane_link.toId > 0:
                            # decide which lane section to use (first or last)
                            if lane_link.fromId < 0:
                                lane_section_idx = (
                                    incoming_road.lanes.getLastLaneSectionIdx()
                                )
                            else:
                                lane_section_idx = 0
                            
                        else:
                            if lane_link.fromId > 0:
                                lane_section_idx = (
                                    incoming_road.lanes.getLastLaneSectionIdx()
                                )
                            else:
                                lane_section_idx = 0
                        incoming_road_id = encode_road_section_lane_width_id(
                                incoming_road.id, lane_section_idx, lane_link.fromId, -1
                            )
                        connecting_road_id = encode_road_section_lane_width_id(
                            connecting_road.id,
                            connecting_road.lanes.getLastLaneSectionIdx(),
                            lane_link.toId,
                            -1,
                        )
                        self.add_link(
                            incoming_road_id, connecting_road_id, lane_link.toId < 0
                        )
                    cur_junction.append(connecting_road_id)
            self.junction_list.append(cur_junction)



    def remove(self, parametric_lane_id):
        """

        Args:
          parametric_lane_id:

        """
        # Delete key
        if parametric_lane_id in self._successors:
            del self._successors[parametric_lane_id]

        # Delete all occurances in successor lists
        for successorsId, successors in self._successors.items():
            if parametric_lane_id in successors:
                self._successors[successorsId].remove(parametric_lane_id)

    def get_successors(self, parametric_lane_id: str) -> list:
        """

        Args:
          parametric_lane_id: Id of ParametricLane for which to search
            successors.

        Returns:
          List of successors belonging the the ParametricLane.
        Par
        """
        if parametric_lane_id not in self._successors:
            return []

        return self._successors[parametric_lane_id]

    def get_predecessors(self, parametric_lane_id: str) -> list:
        """

        Args:
          parametric_lane_id: Id of ParametricLane for which to search predecessors.

        Returns:
          List of predecessors of a ParametricLane.
        """
        # predecessors = []
        # for successor_plane_id, successors in self._successors.items():
        #     if parametric_lane_id not in successors:
        #         continue

        #     if successor_plane_id in predecessors:
        #         continue

        #     predecessors.append(successor_plane_id)

        # return predecessors
        if parametric_lane_id not in self._predecessors:
            return []

        return self._predecessors[parametric_lane_id]
