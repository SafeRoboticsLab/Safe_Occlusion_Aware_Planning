#!/usr/bin/env python3

'''
function to parse common input arguments for carla
'''

import argparse
import carla

''' 
weather preset
'''

SUN_PRESETS = {
    'day': (60.0, 0.0),
    'night': (-90.0, 0.0),
    'sunset': (0.5, 180.0)}

WEATHER_PRESETS = {
    'clear': [10.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.2, 0.0],
    'overcast': [80.0, 0.0, 0.0, 50.0, 2.0, 0.0, 0.9, 10.0],
    'rain': [100.0, 80.0, 90.0, 100.0, 20.0, 0.0, 0.9, 100.0]}

CAR_LIGHTS = {
    'None' : [carla.VehicleLightState.NONE],
    'Position' : [carla.VehicleLightState.Position],
    'LowBeam' : [carla.VehicleLightState.LowBeam],
    'HighBeam' : [carla.VehicleLightState.HighBeam],
    'Brake' : [carla.VehicleLightState.Brake],
    'RightBlinker' : [carla.VehicleLightState.RightBlinker],
    'LeftBlinker' : [carla.VehicleLightState.LeftBlinker],
    'Reverse' : [carla.VehicleLightState.Reverse],
    'Fog' : [carla.VehicleLightState.Fog],
    'Interior' : [carla.VehicleLightState.Interior],
    'Special1' : [carla.VehicleLightState.Special1],
    'Special2' : [carla.VehicleLightState.Special2],
    'All' : [carla.VehicleLightState.All]}

LIGHT_GROUP = {
    'None' : [carla.LightGroup.NONE],
    'Street' : [carla.LightGroup.Street],
    'Building' : [carla.LightGroup.Building],
    'Other' : [carla.LightGroup.Other]}


def PraseArgs():
    argparser = argparse.ArgumentParser(
        description='CARLA Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x960',
        help='window resolution (default: 1280x720)')
    
    # world parameters
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.mercedesccc.mercedesccc',
        help='actor filter (default: "vehicle.mercedesccc.mercedesccc")')
    argparser.add_argument(
        '--map',
        metavar='MAP',
        default='Town01',
        help='name of map (default: "Town01")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='ego',
        help='ego agent name (default: "ego")')
    argparser.add_argument(
        '--dt',
        metavar='TIMESTEP',
        default=0.1,
        help='Synchronous mode time step in second (default: "0.1")')

    # weather parameters
    argparser.add_argument(
        '--sun',
        default='day',
        type=str,
        help='Sun position presets [' + ' | '.join([i for i in SUN_PRESETS]) + ']')
    argparser.add_argument(
        '--weather',
        default='clear',
        type=str,
        help='Weather condition presets [' + ' | '.join([i for i in WEATHER_PRESETS]) + ']')
    argparser.add_argument(
        '--altitude', '-alt',
        metavar='A',
        default=None,
        type=float,
        help='Sun altitude [-90.0, 90.0]')
    argparser.add_argument(
        '--azimuth', '-azm',
        metavar='A',
        default=None,
        type=float,
        help='Sun azimuth [0.0, 360.0]')
    argparser.add_argument(
        '--clouds', '-c',
        metavar='C',
        default=None,
        type=float,
        help='Clouds amount [0.0, 100.0]')
    argparser.add_argument(
        '--rain', '-r',
        metavar='R',
        default=None,
        type=float,
        help='Rain amount [0.0, 100.0]')
    argparser.add_argument(
        '--puddles', '-pd',
        metavar='Pd',
        default=None,
        type=float,
        help='Puddles amount [0.0, 100.0]')
    argparser.add_argument(
        '--wind', '-w',
        metavar='W',
        default=None,
        type=float,
        help='Wind intensity [0.0, 100.0]')
    argparser.add_argument(
        '--fog', '-f',
        metavar='F',
        default=None,
        type=float,
        help='Fog intensity [0.0, 100.0]')
    argparser.add_argument(
        '--fogdist', '-fd',
        metavar='Fd',
        default=None,
        type=float,
        help='Fog Distance [0.0, 100.0)')
    argparser.add_argument(
        '--fogfalloff', '-fo',
        metavar='Fo',
        default=None,
        type=float,
        help='Fog Falloff [0.0, inf)')
    argparser.add_argument(
        '--wetness', '-wet',
        metavar='Wet',
        default=None,
        type=float,
        help='Wetness intensity [0.0, 100.0]')
    argparser.add_argument(
        '--cars',
        metavar='Cars',
        default=None,
        type=str,
        nargs='+',
        help='Light Cars [' + ' | '.join([i for i in CAR_LIGHTS]) + ']')
    argparser.add_argument(
        '--lights', '-l',
        metavar='Lights',
        default=None,
        type=str,
        nargs='+',
        help='Street Lights []')
    argparser.add_argument(
        '--lightgroup', '-lg',
        metavar='LightGroup',
        default=None,
        type=str,
        help='Light Group [' + ' | '.join([i for i in LIGHT_GROUP]) + ']')
    argparser.add_argument(
        '--baseline',
        default=False,
        action="store_true",
        help='Boolean to choose between close loop and baseline planner')
    argparser.add_argument(
        '--obstacle', '-obj',
        default=False,
        action="store_true",
        help='Boolean to determine if obstacle exists')
    
    

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]
    return args

