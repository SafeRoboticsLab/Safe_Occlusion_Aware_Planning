import argparse
import carla

def PraseArgs():
    argparser = argparse.ArgumentParser(
        description='Construct Octomap from pointcloud and camera pose')
    argparser.add_argument(
    '--map',
    metavar='MAP',
    default='Town01',
    help='name of map (default: "Town01")')
    args = argparser.parse_args()

    return args

if __name__ == "__main__":

    args = PraseArgs()

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(20.0)
    client.load_world(args.map)