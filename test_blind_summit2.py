#!/usr/bin/env python3

from client import SynchronousClient, HUD, PraseArgs, Frame, Agent
from mapping import MapViewer
from planning import HybridGame
from recording import RecordUtil




import os


if __name__ == "__main__":
   
    args = PraseArgs()
    args.map = "Town05"
    ego_path = [(34,0,-3, False), (35,0,-3, False), (2035, 0, -3, False), (36,0,-3, False), (37,0,-3, False)]
    

    v_max = 30
    a_max = 8
    dt_sim = 0.05 # sensor are running at this FPS
    dt_plan = 0.5

    save_folder  = os.getcwd()+ "/data/map/"
    filename = save_folder+"Town05_blind_summit"+'.ot'

    record_folder = os.getcwd()+ "/results/planning/"
    record_filename = record_folder+"blind_summit"

    if args.baseline:
        record_filename +="_baseline"
    else:
        record_filename += "_close"
        
    if args.obstacle:
        record_filename += "_obj.pkl"
    else:
        record_filename += "_no_obj.pkl"



    # set up carla client
    client = SynchronousClient(args, dt = dt_sim)
        
    client.setup_ego_vehicle(autopilot = False)
    client.add_lidar(visualize=False, h_fov=360, fps=20)  # when lidar is visualized, the map viewer is dead
    client.add_depth_camera(yaw = 0, name = "0", visualize=False, fov = 110, fps=20)
    client.add_depth_camera(yaw = 90, name = "90", visualize=False, fov = 110, fps=20)
    client.add_depth_camera(yaw = 180, name = "180", visualize=False, fov = 110, fps=20)
    client.add_depth_camera(yaw = -90, name = "-90", visualize=False, fov = 110, fps=20)
    client.ego.set_simulate_physics(False)

    # have a truck moving constant speed
    if args.obstacle:
        truck = Agent(client, [(37,0,-3)], 130, 0)

    # set up planner
    game = HybridGame(client, filename, ego_path, v_max =v_max, a_max=a_max, dp = 0.25, dl = 1, 
                            dt_sim = dt_sim, dt_plan = dt_plan, dv = 0.5, T_replan=0.5, T_plan=5, sensor_range=200)
    game.initialize_game(330, 0, v_max, 150, 15)

    # set up map viewer
    frame = Frame(args.width, args.height)
    hud = HUD(args.width, args.height)
    map_view = MapViewer(args)
    map_view.start(client, follow_ego=False)
    recorder = RecordUtil()

    try:
        for i in range(40):
            client.tick()
            hud.tick(client, game)
            map_view.tick(client)
            client.render()
            map_view.render(frame, shadow_list=None)
            hud.render(frame)
            frame.update_frame()

            # tick all modules
        recorder.add_frame(game)
        running = True
        while running:
            # tick all modules
            client.tick()
            hud.tick(client, game)
            map_view.tick(client)

            # get all sensor data
                        
            semantic_point_cloud = client.lidar_list[0].semantic_raw_data
            sensor_data = [camera.image for camera in client.depth_camera_list]
            sensor_pose = [camera.get_pose() for camera in client.depth_camera_list]

            sensor_data.append(semantic_point_cloud)
            
            running = game.tick(sensor_data, sensor_pose, args.baseline)
            shadow_list = game.info_space.get_shadow_for_plot()
            recorder.add_frame(game)
                            
            client.render()
            map_view.render(frame, shadow_list=shadow_list, dangerzone_list=game.dangerzone_list)
            hud.render(frame)
            frame.update_frame()
        game.plot_hist()
            
            
    finally:
        recorder.save_to_file(record_filename)
        if client:
            client.destroy()