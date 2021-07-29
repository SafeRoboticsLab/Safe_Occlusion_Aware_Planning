import carla
import queue
import numpy as np
from client import SynchronousClient
from mapping import RoadNetwork
from risk_analysis.reachable_set import double_integrator_reachable_tube as double_integrator_FRS
from shapely.geometry import LineString
from carla import Transform
import copy

import heapq


class VehicleRecord():
    """
    Contains information of 
        static:
            Actor pointer in carla world 
                https://carla.readthedocs.io/en/latest/python_api/#carla.Actor
        
        time varying:
            Last timestep this actor is detected by ego
            Last detected bounding box (containing its location and rotation)
            Last detected velocity
    """
    def __init__(self, carla_actor: carla.Vehicle, t, waypoint, a_max = 6, v_min = 0, v_max = 15):
        self.actor = carla_actor
        self.id = self.actor.id
        
        # time varying info 
        self.waypoint = None
        self.last_seen = None # in second
        self.bbox = None # this containing the rotation and tranlation of vehicle center
        self.velocity = None

        self.a_max = a_max
        self.v_min = v_min
        self.v_max = v_max

        self.update(t, waypoint)

        if "carlacola" in self.actor.type_id:
            self.v_max = 15
            self.a_max = 4

    def update(self, t, waypoint):
        self.waypoint = waypoint
        self.last_seen = t
        if self.bbox is None:
            self.bbox = Bbox(self.actor.bounding_box)
        self.bbox.update(waypoint)

        # calculate velocity in Frenet coordinate
        velocity_xyz = self.actor.get_velocity()
        T_w2frenet = np.array(self.waypoint.transform.get_inverse_matrix())[:3,:3]
        v_world = np.array([velocity_xyz.x, velocity_xyz.y, velocity_xyz.z])
        self.velocity = np.round(np.matmul(T_w2frenet, v_world)[0],4)
       
    def predict(self, t_predict):
        """
        t_predict in second
        return a list of waypoints that the vehicle can be at t_predict assuming constant velocity
        """
        s_extent = self.velocity*(t_predict-self.last_seen)
        if s_extent>0:
            return self.velocity, self.waypoint.next(s_extent)
        else:
            return self.velocity, [self.waypoint]

    def calc_FRS(self, v_frenet, delta_t, oppo = False):
        """
        First predict the location of the object at t_predict based on constant velocity model 
        Then calculate its FRS at t_FRS>=t_predict
        """
        if delta_t == 0:
            if oppo:
                FRS = LineString([(- self.bbox.ext_x, -v_frenet), (self.bbox.ext_x, -v_frenet)])
            else:
                FRS = LineString([(- self.bbox.ext_x, v_frenet), (self.bbox.ext_x, v_frenet)])
        else:
            FRS = double_integrator_FRS(v_frenet, v_frenet, delta_t, delta_t, self.a_max, self.v_min, self.v_max, x_extend=self.bbox.ext_x, oppo=oppo)

        return FRS
    
    def state_to_Polygon(self, v_frenet):
        return LineString([(- self.bbox.ext_x, v_frenet), (self.bbox.ext_x, v_frenet)])

    
    
class Bbox():
    def __init__(self, carla_bbox):
        self.ext_x = carla_bbox.extent.x
        self.ext_y = carla_bbox.extent.y
        self.ext_z = carla_bbox.extent.z

       
        dz = carla_bbox.location.z

        self.planes = []

        # self.planes.append(np.array([[0,0,self.ext_z,1], #p0
        #                      [self.ext_x, 0, self.ext_z+dz,1], #p1
        #                      [0, self.ext_y, self.ext_z+dz,1]]).T) #p2

        # self.planes.append(np.array([[0,0,-self.ext_z+dz,1], #p0
        #                         [self.ext_x, 0, -self.ext_z+dz,1], #p1
        #                         [0, self.ext_y, -self.ext_z+dz,1]]).T) #p2

        self.planes.append(np.array([[self.ext_x, 0, 0,1], #p0
                                [self.ext_x, self.ext_y, 0,1], #p1
                                [self.ext_x, 0, self.ext_z+dz,1]]).T) #p2

        self.planes.append(np.array([[-self.ext_x, 0, 0,1], #p0
                                [-self.ext_x, self.ext_y, 0,1], #p1
                                [-self.ext_x, 0, self.ext_z+dz,1]]).T) #p2

        self.planes.append(np.array([[0, self.ext_y, 0,1], #p0
                                [self.ext_x, self.ext_y, 0,1], #p1
                                [0, self.ext_y, self.ext_z+dz,1]]).T) #p2

        self.planes.append(np.array([[0, -self.ext_y, 0,1], #p0
                                [self.ext_x, -self.ext_y, 0,1], #p1
                                [0, -self.ext_y, self.ext_z+dz,1]]).T) #p2

        self.planes_world = None

    def update(self, waypoint):
        T_v2w = np.array(waypoint.transform.get_matrix())
        self.planes_world = [np.dot(T_v2w, plane) for plane in self.planes]


    def check_occlusion(self, sensor_w, pt_w):
        """
        sensor_w = [x,y,z]
        pt_w = [x,y,z]
        T_v2w = SE(3) transform vehicle center to the world frame
        """
        num_intersect = 0
        for plane_w in self.planes_world:
            if self.plane_lane_intersect(sensor_w, pt_w, plane_w):
                num_intersect += 1
                if num_intersect == 2:
                    return num_intersect
        return num_intersect

        
    @staticmethod
    def plane_lane_intersect(la, lb, surface):
        """
            la = [la_x, la_y, la_z]
            lb = [lb_x, lb_y, lb_z]

            surface = [[p0_x, p1_x, p2_x],
                        [p0_y, p1_y, p2_y],
                        [p0_z, p1_z, p2_z]]

        """

        A11 = la[0]-lb[0] #la_x - lb_x
        A12 = surface[0,1] - surface[0,0] #p1_x - p0_x
        A13 = surface[0,2] - surface[0,0] #p2_x - p0_x

        A21 = la[1]-lb[1] #la_y - lb_y
        A22 = surface[1,1] - surface[1,0] #p1_y - p0_y
        A23 = surface[1,2] - surface[1,0] #p2_y - p0_y

        A31 = la[2]-lb[2] #la_z - lb_z
        A32 = surface[2,1] - surface[2,0] #p1_z - p0_z
        A33 = surface[2,2] - surface[2,0] #p2_z - p0_z

        A = np.array([[A11, A12, A13], [A21, A22, A23], [A31, A32, A33]])
        b = np.array([la[0] - surface[0,0],
                        la[1] - surface[1,0],
                        la[2] - surface[2,0]])

        det = np.linalg.det(A)
        if -1e-5<det<1e-5:
            return False
                
        Ai11 = A22*A33 - A23*A32
        Ai12 = A13*A32 - A12*A33
        Ai13 = A12*A23 - A13*A22
        Ai21 = A23*A31 - A21*A33
        Ai22 = A11*A33 - A13*A31
        Ai23 = A13*A21 - A23*A11
        Ai31 = A21*A32 - A22*A31
        Ai32 = A12*A31 - A11*A32
        Ai33 = A11*A22 - A12*A21

        Ai = np.array([[Ai11, Ai12, Ai13], [Ai21, Ai22, Ai23], [Ai31, Ai32, Ai33]])/det
        
        x = np.dot(Ai, b)
        if -1e-3<=x[0]<=1 and -1<=x[1]<=1 and -1<=x[2]<=1:
            return True
        else:
            return False
        
class ObjectUtils():
    def __init__(self, client: SynchronousClient, road_network: RoadNetwork):
        self.client = client
        self.ego_id = self.client.ego.id # to be ignored during update
        self.road_network = road_network
        self.obstacle_list = {}
        # a look up table. 
        # Given a time step, generate a map of predicted obstacle and bounding box

        # dict:
        #    key: t in second
        #    items: dict of 
        #                key: lanletif
        #                items: priority queue of (ds, v_frenet, VehicleRecord )
 
        self.predicted_obstacle_map = {}

        # dict:
        #    key: t in second
        #    items: list of Bbox class

        self.predicted_bbox_table = {}

        # dict:
        #    key: t in second
        #    items: map key: obstalce id, item list of occupied laneletid

        self.predicted_occupied_lanlet_map = {}


    def reset_prediction(self):
        self.predicted_obstacle_map = {}
        self.predicted_bbox_table = {}
        self.predicted_occupied_lanlet_map = {}

        
    def update_obstacles(self, t, semantic_point_cloud):
        # We assume that only obstacles on roads are other vehicles and ignore predrestrains and other objects
        # In addition, we use the ground truth information from carla to update the belief
        # 
        # TODO 
        #   1) Implement some real-world object detection and tracking algorithms here from raw lidar/camera data
        #   2) Add objects other than vehicles
        #   3) How to deal with objects that lost track (delete/predict)
        
        # filter out the data points belongs to a vehcile
        filterd_data = semantic_point_cloud[semantic_point_cloud['ObjTag']==10]
        observed_obj_list = np.unique(filterd_data['ObjIdx'])
        # In the road network, update the waypoints occupied by the ego vehicle as seen
        waypoint = self.client.map.get_waypoint(self.client.ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        self.road_network.update_state_batch(t, waypoint.road_id, waypoint.section_id, waypoint.lane_id, waypoint.s, 
                    [self.client.ego.bounding_box.extent.x+1, self.client.ego.bounding_box.extent.x+1], from_end=False, ds_global=True)

        for obj in observed_obj_list:
            if obj == 0 or obj == self.ego_id:
                continue # what is 0?
            obj = int(obj)
            
            if obj not in self.obstacle_list:
                actor = self.client.world.get_actor(obj)
                waypoint = self.client.map.get_waypoint(actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
                self.obstacle_list[obj] = VehicleRecord(actor, t, waypoint)
            else:
                actor = self.obstacle_list[obj].actor
                waypoint = self.client.map.get_waypoint(actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
                self.obstacle_list[obj].update(t, waypoint)


                       
            # In the road network, update the waypoints occupied by the vehicle as seen
            self.road_network.update_state_batch(t, waypoint.road_id, waypoint.section_id, waypoint.lane_id, waypoint.s, 
                    [actor.bounding_box.extent.x+1, actor.bounding_box.extent.x+1], from_end=False, ds_global=True)

    def predict_obstacle(self, t_predict, use_record = False):
        # key is lanelet_id
        # items is a priority queue of tuples containing (sorted by ds)
        #   1. predict ds
        #   2. predicted speed
        #   3. shapely Polygon of 3d Bounding box
        #   4. obstacle itself
        
        if t_predict not in self.predicted_obstacle_map or not use_record:
            
            obstacle_map = {}
            occupied_lanelet_map = {} # given obstacle id, find occupied lanelet
            # obstacle_map
            for obstacle in self.obstacle_list.values():
                v_frenet, waypoint_list = obstacle.predict(t_predict)
                obstacle_id = obstacle.id
                occupied_lanelet_map[obstacle_id] = []
                for waypoint in waypoint_list:
                    lane = self.road_network.find_lane(waypoint.road_id, waypoint.section_id, waypoint.lane_id)
                    lanelet_id = lane.lanelet_id
                    occupied_lanelet_map[obstacle_id].append(lanelet_id)
                    if lanelet_id not in obstacle_map:
                        obstacle_map[lanelet_id] = []
                    ds = waypoint.s - lane.lane_section.s_start
                    heapq.heappush(obstacle_map[lanelet_id], (ds, v_frenet, obstacle))
            self.predicted_obstacle_map[t_predict] = obstacle_map
            self.predicted_occupied_lanlet_map[t_predict] = occupied_lanelet_map
            return obstacle_map, occupied_lanelet_map
        else:
            return self.predicted_obstacle_map[t_predict], self.predicted_occupied_lanlet_map[t_predict]

    def predict_bbox(self, t_predict, use_record = True):
        # Assume constant velcoity, predict the ist of bbox at t_predict 
        if t_predict not in self.predicted_bbox_table or not use_record:
            predicted_bbox_list = []
            # obstacle_map
            for obstacle in self.obstacle_list.values():
                _, waypoint_list = obstacle.predict(t_predict )
                for waypoint in waypoint_list:
                    # predict bbox
                    bbox = copy.deepcopy(obstacle.bbox)
                    bbox.update(waypoint)
                    predicted_bbox_list.append(bbox)
            self.predicted_bbox_table[t_predict] = predicted_bbox_list
            return predicted_bbox_list
        else:
            return self.predicted_bbox_table[t_predict]

    


    


    