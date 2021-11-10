#!/usr/bin/env python3


from client import SynchronousClient, PraseArgs
import pickle, os







if __name__ == "__main__":
   
    args = PraseArgs()
    args.map = "Town05"

    folder  = os.getcwd()+'/data/depth/'
    filename = args.map+'_blind_summit'

    

    client = None
    try:
        # set up carla client
        client = SynchronousClient(args)
        
        #ego_path = [(24,0,1,False), (272,0,1,False), (23,0,1,False), (330,0,1, False), (22,0,1, False)]
        ego_path = [(34,0,-2, True), (35,0,-2, True), (2035, 0, -2, True), (36,0,-2, True), (37,0,-2, True)]
        #ego_path = [(35,0,-2, True), (2035, 0, -2, True), (36,0,-2, True)]
        
        waypoint_list = []

        def add_waypoint(id, start = 1, ds=1):
            local_waypoints = []
            waypoint_start = client.map.get_waypoint_xodr(id[0], id[2], start)
            if waypoint_start is None:
                print(id)
            #local_waypoints.append(waypoint_start)
            if id[2]>0:
                local_waypoints = waypoint_start.previous_until_lane_start(ds)
                local_waypoints.reverse()
                waypoint_list.extend(local_waypoints)
            else:
                local_waypoints = waypoint_start.next_until_lane_end(ds)
                waypoint_list.extend(local_waypoints)

        #add_waypoint(ego_path[0], 200, 0.5)
        add_waypoint(ego_path[3],0, 0.5)
        add_waypoint(ego_path[4],0, 0.5)
        #add_waypoint(ego_path[2],0, 0.5)
        waypoint_list = waypoint_list[:150]
        spwan_point = waypoint_list[0].transform
        spwan_point.location.z += 0.2
        
        client.setup_ego_vehicle(autopilot = False, pose=spwan_point)
        client.add_depth_camera(width=640, height=480, x=2.5, z =1.8, yaw = 0, name = "0", visualize=False, fov = 110)
        client.add_depth_camera(width=640, height=480, y=1, z =1.8, yaw = 90, name = "90", visualize=False, fov = 110)
        #client.add_depth_camera(width=640, height=480, x=-2.5, z =1.8, yaw = 180, name = "180", visualize=False, fov = 110)
        client.add_depth_camera(width=640, height=480, y=-1, z =1.8, yaw = -90, name = "-90", visualize=False, fov = 110)
        client.ego.set_simulate_physics(False)

        point_cloud_list = []

        for _ in range(10):
            client.tick()
            client.render()

        print(len(waypoint_list), "waypoints in total")
        for waypoint in waypoint_list:
            client.ego.set_transform(waypoint.transform)
            client.tick()
            
            client.render()
           
            for i in range(3):
                point_cloud = client.depth_camera_list[i].get_point_cloud(75)
                if point_cloud is not None:
                    camera_pose = client.depth_camera_list[i].get_pose()
                    pose = [camera_pose.location.x, camera_pose.location.y, camera_pose.location.z,
                            camera_pose.rotation.roll, camera_pose.rotation.pitch, camera_pose.rotation.yaw]
                    point_cloud_list.append((point_cloud, pose))
            
        with open(folder+filename+'.pkl',"wb") as f:
            #pickle.dump( (env, plan_set, valid_plan_set), f, pickle.HIGHEST_PROTOCOL)
            pickle.dump( point_cloud_list, f, pickle.HIGHEST_PROTOCOL)
        print("Save "+str(len(point_cloud_list))+" to "+folder+filename+".pkl")


    finally:
        if client:
            client.destroy()

    
    