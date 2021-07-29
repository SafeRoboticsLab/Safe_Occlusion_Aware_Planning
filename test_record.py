#!/usr/bin/env python3

from client import SynchronousClient, HUD, PraseArgs, Frame, Agent
from mapping import MapViewer
from recording import Record
from risk_analysis import ISpace
import time
import numpy as np
import os
import pickle

if __name__ == "__main__":
    args = PraseArgs()
    
    file_name = None
    # load file
    file = open(file_name,'rb')
    record = pickle.load(file)
    file.close()

    args.map = record.map

    client = SynchronousClient(args)
    info_space = ISpace(client, None, [])

    frame = Frame(args.width, args.height)
    hud = HUD(args.width, args.height)
    map_view = MapViewer(args)
    map_view.start(client, follow_ego=True)


    for i in range(len(record.frame_list)):
        shadow_list = record.replay_frame(i, client)
        client.tick()
        map_view.tick(client)
            
        shadow_list = info_space.get_shadow_for_plot(shadow_list)
        client.render()
        map_view.render(frame, shadow_list=shadow_list)
        hud.render(frame)
        frame.update_frame()