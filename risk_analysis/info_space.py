#!/usr/bin/env python3

import queue
from datetime import datetime
import sys
import numpy as np
import carla
from mapping import RoadNetwork, PlaneMap
from risk_analysis.shadow_utils import ShadowUtils
from risk_analysis.object_utils import ObjectUtils
from risk_analysis.state_checker import StateChecker
from planning import Route 
from pympler.asizeof import asizeof
import queue

class ISpace():
    def __init__(self, client, octomap_file, ego_path=None, range = 100, ds = 0.5, v_max = 20, a_max = 6):
        # carla client
        self.client = client
        # road network    
        self.road_network = RoadNetwork(client)
        # 2D map for visualization 
        self.plane_map = PlaneMap(client)
        # recrod route that the ego vehicle need to take
        self.route = Route(self.client, self.road_network, ego_path)
        # class that updates and predict occlusion
        self.shadow_manager = ShadowUtils(self.client, self.road_network, octomap_file, self.route, range, ds, v_max, a_max)
        # class that records, and predicts objects in the scene
        self.object_manager = ObjectUtils(self.client, self.road_network)
        # class that check if a planned trajectory is safe 
        self.state_checker = StateChecker(self.client, self.road_network, self.route, self.shadow_manager, v_max, a_max)


    def reset_prediction(self):
        self.object_manager.reset_prediction()


    def update(self, t, sensor_data, sensor_pose):
        """
        Given a point cloud and a sensor transform matrix, update the information space
        Input:
            time_step: int (1,2,3....)
            sensor_data: (list of) images/ lidar point clouds
            sensor_pose: list of carla.Transform of lidar sensor            
            range: max line of sight consider visible
        """
        # t0 = datetime.now()

        # update the obstacles in the current FOV
        self.object_manager.update_obstacles(t, sensor_data[-1])
        shadow_map, shadow_list = self.shadow_manager.update_visibility(sensor_data[:-1], sensor_pose, t)
                
        # # time analysis
        # t_last = datetime.now() - t0
        # sys.stdout.write('\r FPS: ' + str(1.0 / t_last.total_seconds()))
        # sys.stdout.flush()

        return shadow_map, shadow_list

    def update_octomap(self, t, sensor_data, sensor_pose):
        """
        Given a point cloud and a sensor transform matrix, update the information space
        Input:
            time_step: int (1,2,3....)
            sensor_data: (list of) images/ lidar point clouds
            sensor_pose: list of carla.Transform of lidar sensor            
            range: max line of sight consider visible
        """
        # t0 = datetime.now()

        # update the obstacles in the current FOV
        self.object_manager.update_obstacles(t, sensor_data[-1])
        predicted_bbox_list = self.object_manager.predict_bbox(t)
        pose = sensor_pose[0].location
        shadow_map, shadow_list = self.shadow_manager.update_visibility_octomap(predicted_bbox_list, [pose.x, pose.y, pose.z], t)
                
        # # time analysis
        # t_last = datetime.now() - t0
        # sys.stdout.write('\r FPS: ' + str(1.0 / t_last.total_seconds()))
        # sys.stdout.flush()

        return shadow_map, shadow_list

    def predict(self, t_predict, old_shadow, ego_pose, use_record = True):
        #t0 = datetime.now()
        t_predict = t_predict
        predicted_bbox_list = self.object_manager.predict_bbox(t_predict, use_record)
        predicted_shadow_map, predicted_shadow_list = self.shadow_manager.predict_visibility(old_shadow, predicted_bbox_list, ego_pose, t_predict)


        # time analysis
        # t_last = datetime.now() - t0
        # sys.stdout.write('\r Prediction FPS: ' + str(1.0 / t_last.total_seconds()))
        # sys.stdout.flush()

        return predicted_shadow_map, predicted_shadow_list



    def get_shadow_for_plot(self, shadow_to_plot = None):
        return self.shadow_manager.get_shadow_for_plot(shadow_to_plot)
                     
    




