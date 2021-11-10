from mapping import OccupancyMap
import pickle, sys
from datetime import datetime
import argparse

def PraseArgs():
    argparser = argparse.ArgumentParser(
        description='Construct Octomap from pointcloud and camera pose')
    argparser.add_argument(
    '--map',
    metavar='MAP',
    default='Town01',
    help='name of map (default: "Town01")')
    argparser.add_argument(
    '--res',
    metavar='RESOLUTION',
    default= 0.25,
    type = float,
    help='Resolution of octomap (default: 0.5)')
    args = argparser.parse_args()

    return args

if __name__ == "__main__":

    args = PraseArgs()

    save_folder = "data/map/"
    pointcloud_folder = "data/depth/"
    #filename = "Town01_intersection2"
    filename = 'Town05_blind_summit'
    infile = open(pointcloud_folder+filename+'.pkl','rb')
    pc_list = pickle.load(infile)
    infile.close()

    occupancy_map = OccupancyMap(filename=save_folder+filename+'.ot',resolution = args.res)
    #occupancy_map = OccupancyMap(resolution = args.res)
    print("Load ", len(pc_list), "pointclouds")

    dt0 = datetime.now()
    for (pc, pose) in pc_list:
        occupancy_map.insert_point_cloud(pc, pose, range=500)
        process_time = datetime.now() - dt0
        sys.stdout.write('\r FPS: ' + str(1.0 / process_time.total_seconds()))
        sys.stdout.flush()
        dt0 = datetime.now()
    

    print("\n Finish pointcloud inserting")

    #occupancy_map.visualize()
    occupancy_map.write_to_file(save_folder+filename+'.ot')


    # occu = OccupancyMap(save_folder+filename+'.ot')
    # occu.write_to_file(save_folder+filename+'.ot')

    
   
