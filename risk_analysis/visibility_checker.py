from mapping import OccupancyMap
import numpy as np
from mapping.road_network import Lane
from shapely.geometry import LineString
class OctomapVisibility():
    def __init__(self, octomap_file, road_network, range ):
        # load octomap
        self.occupancy_map = OccupancyMap(filename = octomap_file)
        self.road_network = road_network
        self.range = range


    def check_visibility(self, lane: Lane, ds, bbox_list, sensor_loc, time_step):
        # sensor_loc [x,y,z]
        
        idx = np.argmin(np.abs(lane.waypoint_ds-ds))
        left_w = lane.left_vertices[idx,:] # [x,y,z,1] of waypoint in the world frame
        right_w = lane.right_vertices[idx,:]
        center_w = lane.center_vertices[idx,:]
        # num_vis_center = 0
        # check center can be visible
        for dz in [0.5, 1.2]:
            pt_w = np.array([center_w[0],center_w[1],center_w[2]+dz,1]).T
            if self._check_pt_visibility(sensor_loc, pt_w, bbox_list):
                return True
        #         num_vis_center += 1
            
        # if num_vis_center == 0:
        #     return False
                
        # for r in [0.2, 0.8]:
        #     x = r*left_w[0]+(1-r)*right_w[0]
        #     y = r*left_w[1]+(1-r)*right_w[1]
        #     z = r*left_w[2]+(1-r)*right_w[2]
        #     for dz in [0.5, 1.2]:
        #         pt_w = np.array([x,y,z+dz,1]).T
        #         if self._check_pt_visibility(sensor_loc, pt_w, bbox_list):
        #             return True
        return False
    
    def _check_pt_visibility(self, sensor_loc, pt_w, bbox_list):
        # first check range
        dx = pt_w[0]-sensor_loc[0]
        dy = pt_w[1]-sensor_loc[1]
        dz = pt_w[2]-sensor_loc[2]
        dis = np.sqrt(dx**2+dy**2+dz**2)
        if dis > self.range:
            return False

        # check octomap
        if not self.occupancy_map.check_visibility(sensor_loc, pt_w):
            return False
        
        for bbox in bbox_list:
            num_intersection = bbox.check_occlusion(sensor_loc, pt_w)
            if num_intersection>1:
                return False

        return True
              
class DepthVisibility():
    def __init__(self, road_network, range, img_width = 1920, img_height = 1080, h_fov = 110):
        # load octomap
        self.road_network = road_network
        self.range = range
        self.T_c2img = np.array([[0, 1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
        self.K  = np.zeros((3,4))
        self.K[:,:3] = np.identity(3)
        self.K[0, 2] = img_width / 2.0
        self.K[1, 2] = img_height / 2.0
        self.K[0, 0] = self.K[1, 1] = img_width / (2.0 * np.tan(h_fov * np.pi / 360.0))
        self.Tw2img_list = [None, None, None, None]
        self.t_list = [-1, -1, -1, -1]
        self.img_height = img_height


    def check_visibility(self, lane: Lane, ds, img_list, sensor_pose_list , time_step):
        """
        We have four depth camera
        D1 covers (-45,45]deg hfov w.r.t vehicle frames
        D2 covers (45, 135]deg hfov w.r.t vehicle frames
        D3 covers (135, 225]deg hfov w.r.t vehicle frames
        D3 covers (225, 315]deg hfov w.r.t vehicle frames
        """
        
        # # # check if I can use previous test
        if ds in lane.waypoint_map:
            time, visible = lane.waypoint_map[ds]
            if time == time_step:
                return visible

        idx = np.argmin(np.abs(lane.waypoint_ds-ds))
        left_w = lane.left_vertices[idx,:] # [x,y,z,1] of waypoint in the world frame
        right_w = lane.right_vertices[idx,:]
        center_w = lane.center_vertices[idx,:]
        
        # check center can be visible
        #num_vis_center = 0
        for dz in [0.5, 1.2]:
            pt_w = np.array([center_w[0],center_w[1],center_w[2]+dz,1]).T
            if self._check_pt_visibility(pt_w, img_list, sensor_pose_list , time_step):
                return True
        #         num_vis_center += 1
            
        # if num_vis_center == 0:
        #     return False
                
        # for r in [0.2, 0.8]:
        #     x = r*left_w[0]+(1-r)*right_w[0]
        #     y = r*left_w[1]+(1-r)*right_w[1]
        #     z = r*left_w[2]+(1-r)*right_w[2]
        #     for dz in [0.5, 1.2]:
        #         pt_w = np.array([x,y,z+dz,1]).T
        #         if self._check_pt_visibility(pt_w, img_list, sensor_pose_list , time_step):
        #             return True
        return False

    def _check_pt_visibility(self, pt_w, img_list, sensor_pose_list , time_step):
        """
        pt_w = np.array([x,y,z,1])
        """
        ref_pose = sensor_pose_list[0] # since the first camera align with the vehicle reference frame
        waypoint_v = np.matmul(np.array(ref_pose.get_inverse_matrix()), pt_w.T)
        dis = np.linalg.norm(waypoint_v[:3])
        if dis<5:
            return True
        elif dis<self.range:
            theta = np.arctan2(waypoint_v[1], waypoint_v[0]) # waypoint's azimuth in vehicle frame in -pi to pi
            if -np.pi/4<theta<=np.pi/4:
                cam_idx = 0
            elif np.pi/4<theta<=3*np.pi/4:
                cam_idx = 1
            elif theta>3*np.pi/4 or theta<=-3*np.pi/4:
                cam_idx = 2
            else:
                cam_idx = 3

            active_img = img_list[cam_idx]
            if self.t_list[cam_idx]< time_step:
                active_pose = sensor_pose_list[cam_idx]
                T_w2img = np.matmul(self.T_c2img, np.array(active_pose.get_inverse_matrix()))
                active_Tw2img = np.matmul(self.K, T_w2img)
                self.Tw2img_list[cam_idx] = active_Tw2img
                self.t_list[cam_idx] = time_step
            else:
                active_Tw2img = self.Tw2img_list[cam_idx]
            pt_uvz = np.matmul(active_Tw2img, pt_w)
            pt_uv = np.round(pt_uvz/pt_uvz[2])
            if pt_uv[1]<0 or pt_uv[1]>=self.img_height:
                return False
            if pt_uvz[2]<=active_img[int(pt_uv[1]), int(pt_uv[0])]:
                return True
            else:
                return False
                
           

                            






        
