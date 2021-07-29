#!/usr/bin/env python

"""
Script to control weather parameters in simulations

Opts:

    -h, --help                  show this help message and exit
    --host H                    IP of the host server (default: 127.0.0.1)
    -p P, --port P              TCP port to listen to (default: 2000)
    --sun SUN                   Sun position presets [sunset | day | night]
    --weather WEATHER           Weather condition presets [clear | overcast | rain]
    --altitude A, -alt A        Sun altitude [-90.0, 90.0] When Sun altitude <0, the night mode begin
    --azimuth A, -azm A         Sun azimuth [0.0, 360.0]
    --clouds C, -c C            Clouds amount [0.0, 100.0]
    --rain R, -r R              Rain amount [0.0, 100.0]
    --puddles Pd, -pd Pd        Puddles amount [0.0, 100.0]
    --wind W, -w W              Wind intensity [0.0, 100.0]
    --fog F, -f F               Fog intensity [0.0, 100.0]
    --fogdist Fd, -fd Fd        Fog Distance [0.0, inf)
    --wetness Wet, -wet Wet     Wetness intensity [0.0, 100.0]
"""

import sys
import carla

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

def apply_sun_presets(args, weather):
    """Uses sun presets to set the sun position"""
    if args.sun is not None:
        if args.sun in SUN_PRESETS:
            weather.sun_altitude_angle = SUN_PRESETS[args.sun][0]
            weather.sun_azimuth_angle = SUN_PRESETS[args.sun][1]
        else:
            print("[ERROR]: Command [--sun | -s] '" + args.sun + "' not known")
            sys.exit(1)


def apply_weather_presets(args, weather):
    """Uses weather presets to set the weather parameters"""
    if args.weather is not None:
        if args.weather in WEATHER_PRESETS:
            weather.cloudiness = WEATHER_PRESETS[args.weather][0]
            weather.precipitation = WEATHER_PRESETS[args.weather][1]
            weather.precipitation_deposits = WEATHER_PRESETS[args.weather][2]
            weather.wind_intensity = WEATHER_PRESETS[args.weather][3]
            weather.fog_density = WEATHER_PRESETS[args.weather][4]
            weather.fog_distance = WEATHER_PRESETS[args.weather][5]
            weather.fog_falloff = WEATHER_PRESETS[args.weather][6]
            weather.wetness = WEATHER_PRESETS[args.weather][7]
        else:
            print("[ERROR]: Command [--weather | -w] '" + args.weather + "' not known")
            sys.exit(1)


def apply_weather_values(args, weather):
    """Set weather values individually"""
    if args.azimuth is not None:
        weather.sun_azimuth_angle = args.azimuth
    if args.altitude is not None:
        weather.sun_altitude_angle = args.altitude
    if args.clouds is not None:
        weather.cloudiness = args.clouds
    if args.rain is not None:
        weather.precipitation = args.rain
    if args.puddles is not None:
        weather.precipitation_deposits = args.puddles
    if args.wind is not None:
        weather.wind_intensity = args.wind
    if args.fog is not None:
        weather.fog_density = args.fog
    if args.fogdist is not None:
        weather.fog_distance = args.fogdist
    if args.fogfalloff is not None:
        weather.fog_falloff = args.fogfalloff
    if args.wetness is not None:
        weather.wetness = args.wetness


def apply_lights_to_cars(args, world):
    if args.cars is None:
        return

    light_mask = carla.VehicleLightState.NONE
    for option in args.cars:
        light_mask |= CAR_LIGHTS[option][0]

    # Get all cars in level
    all_vehicles = world.get_actors()
    for ve in all_vehicles:
        if "vehicle." in ve.type_id:
            ve.set_light_state(carla.VehicleLightState(light_mask))

def apply_lights_manager(args, light_manager):
    if args.lights is None:
        return

    light_group = 'None'
    if args.lightgroup is not None:
        light_group = args.lightgroup

    # filter by group
    lights = light_manager.get_all_lights(LIGHT_GROUP[light_group][0]) # light_group

    i = 0
    while (i < len(args.lights)):
        option = args.lights[i]

        if option == "on":
            light_manager.turn_on(lights)
        elif option == "off":
            light_manager.turn_off(lights)
        elif option == "intensity":
            light_manager.set_intensity(lights, int(args.lights[i + 1]))
            i += 1
        elif option == "color":
            r = int(args.lights[i + 1])
            g = int(args.lights[i + 2])
            b = int(args.lights[i + 3])
            light_manager.set_color(lights, carla.Color(r, g, b))
            i += 3

        i += 1


def set_weather(world, args):
    """Start function"""
    
    # since all the arguments are None by default
    # (except for the first 2, host and port)
    # we can check if all the arguments have been provided

    weather = world.get_weather()

    # apply presets
    apply_sun_presets(args, weather)
    apply_weather_presets(args, weather)

    # apply weather values individually
    apply_weather_values(args, weather)

    world.set_weather(weather)

    # apply car light changes
    apply_lights_to_cars(args, world)

    apply_lights_manager(args, world.get_lightmanager())

    world.wait_for_tick()
