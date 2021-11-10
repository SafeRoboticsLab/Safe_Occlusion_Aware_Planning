#!/usr/bin/env python3

from client import SynchronousClient, HUD, PraseArgs, Frame, Agent
from mapping import MapViewer
from recording import RecordUtil
from risk_analysis import ISpace
import time
import numpy as np
import os
import pickle
import time

if __name__ == "__main__":

    filename = "blind_summit_close_no_obj"
    args = PraseArgs()
    args.map = 'Town05'
    record_folder = os.getcwd()+ "/results/Good/"
    record_filename = record_folder+filename+".pkl"
    #"interseciton_close_no_obj.pkl"
    #"interseciton_close_obj.pkl"
    #"overtake_close_obj.pkl"
    #"overtake_close_no_obj.pkl"
    #"overtake_baseline_no_obj.pkl"
    #"interseciton_baseline_no_obj.pkl"

    video_folder = os.getcwd()+ "/results/video/BlindSummit/"
    
    
    recorder = RecordUtil()
    recorder.load_from_file(record_filename)
    dt = 0.05


    client = SynchronousClient(args, dt =dt )
    info_space = ISpace(client, None, [])

    frame = Frame(args.width, args.height)
    hud = HUD(args.width, args.height)
    map_view = MapViewer(args)
    map_view.start(client, follow_ego=True)

    try:
    
        shadow_list, danger_zone, cur_frame = recorder.replay_frame(0, client)
        client.setup_spectator()
        client.add_RGB_camera(visualize=True, fps=20, fov=100)
        #client.add_lidar(fps=20, h_fov=360)

        num_frame = recorder.num_frame
        for _ in range(100):
            client.tick()
            hud.tick_record(cur_frame)
            map_view.tick(client)
                
            shadow_plot = info_space.get_shadow_for_plot(shadow_list)
            client.render()
            map_view.render(frame, shadow_list=shadow_plot, dangerzone_list=danger_zone)
            hud.render(frame)
            frame.update_frame()

        for i in range(num_frame):
            shadow_list, danger_zone, cur_frame = recorder.replay_frame(i, client)
            client.tick()
            hud.tick_record(cur_frame)
            map_view.tick(client)
                
            shadow_plot = info_space.get_shadow_for_plot(shadow_list)
            client.render(video_folder, i)
            map_view.render(frame, shadow_list=shadow_plot, dangerzone_list=danger_zone)
            hud.render(frame)
            frame.update_frame()
            
            # saving
            frame.save_frame(video_folder, i)
           

    finally:
        client.destroy()
        