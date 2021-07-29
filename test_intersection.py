#!/usr/bin/env python3

from client import SynchronousClient, HUD, PraseArgs, Frame, Agent
from mapping import MapViewer
from planning import HybridGame
import time
import numpy as np
import os


if __name__ == "__main__":
   
    args = PraseArgs()
    args.map = "Town01"
    #ego_path = [(17, 0, -1, False), (296, 0, -1, False), (18, 0, -1, False), (218, 0, -1, False), (19, 0, -1, False)]
    ego_path = [(24,0,1,False), (272,0,1,False), (23,0,1,False), (330,0,1, False), (22,0,1, False), (179,0,1, False), (21,0,1, False)]
    #args.map = "Town07"
    #ego_path = [(37,0,1,False), (188,0,1,False), (36,0,1,False)]
    
    v_max = 15
    a_max = 6
    dt_sim = 0.1 # sensor are running at this FPS
    dt_plan = 0.5

    save_folder  = os.getcwd()+ "/data/map/"
    filename = save_folder+args.map+"_intersection2"+'.ot'

    client = None
    try:
        # set up carla client
        client = SynchronousClient(args, dt = dt_sim)
           
        client.setup_ego_vehicle(autopilot = False)
        client.add_lidar(visualize=False, h_fov=360)  # when lidar is visualized, the map viewer is dead
        client.add_depth_camera(yaw = 0, name = "0", visualize=False, fov = 110)
        client.add_depth_camera(yaw = 90, name = "90", visualize=False, fov = 110)
        client.add_depth_camera(yaw = 180, name = "180", visualize=False, fov = 110)
        client.add_depth_camera(yaw = -90, name = "-90", visualize=False, fov = 110)
        client.ego.set_simulate_physics(False)

        # have a truck moving constant speed
        truck = Agent(client, [(4,0,-1)], 10, 0)
        truck2 = Agent(client, [(4,0,-1)], 15.4, 0)
        #obstacle = Agent(client, [(4,0,1), (23,0,-1)], 14, 10, 'vehicle.bmw.isetta')

        # set up planner
        game = HybridGame(client, filename, ego_path, v_max =v_max, a_max=a_max,  dp = 0.25, dl = 0.5, dt_sim = dt_sim, dt_plan = dt_plan, dv = 0.5, T_replan = 2.5)
        game.initialize_game(120, 0, 10)

        # set up map viewer
        frame = Frame(args.width, args.height)
        hud = HUD(args.width, args.height)
        map_view = MapViewer(args)
        map_view.start(client, follow_ego=True)

        for i in range(10):
            #truck.tick()
            client.tick()
            hud.tick(client, game)
            map_view.tick(client)
            client.render()
            map_view.render(frame, shadow_list=None)
            hud.render(frame)
            frame.update_frame()
        input("Press Enter to continue...")


        for i in range(100):
            # tick all modules
            #obstacle.tick()
            
            client.tick()
            hud.tick(client, game)
            map_view.tick(client)

            # get all sensor data
                        
            semantic_point_cloud = client.lidar_list[0].semantic_raw_data
            sensor_data = [camera.image for camera in client.depth_camera_list]
            sensor_pose = [camera.get_pose() for camera in client.depth_camera_list]

            sensor_data.append(semantic_point_cloud)
            
            game.tick(sensor_data, sensor_pose, False)
            shadow_list = game.info_space.get_shadow_for_plot(game.predicted_shadow)
            client.render()

            map_view.render(frame, shadow_list=shadow_list)
            hud.render(frame)
            frame.update_frame()
            #input("Press Enter to continue...\\")
        game.plot_hist()
            
    finally:
        if client:
            client.destroy()