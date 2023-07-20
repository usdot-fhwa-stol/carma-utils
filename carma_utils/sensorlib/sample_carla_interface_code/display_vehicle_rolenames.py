import glob
import os
import sys
import time

from find_carla_egg import find_carla_egg

carla_egg_file = find_carla_egg()

sys.path.append(carla_egg_file)

import carla

import argparse

argparser = argparse.ArgumentParser(
    description=__doc__)
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
    '--filterv',
    metavar='PATTERN',
    default='vehicle.*',
    help='vehicles filter (default: "vehicle.*")')
argparser.add_argument(
    '--filterw',
    metavar='PATTERN',
    default='walker.pedestrian.*',
    help='pedestrians filter (default: "walker.pedestrian.*")')
argparser.add_argument(
    '--sync',
    action='store_true',
    help='Synchronous mode execution')
argparser.add_argument(
    '-d', '--duration',
    metavar='D',
    default=10,
    type=int,
    help='duration to display vehicle rolenames - use 0 for indefinite (default: 10)')

args = argparser.parse_args()


try:
    client = carla.Client(args.host, 2000)
    client.set_timeout(5.0)
    

    while (True):
        world = client.get_world()
        # Get actor information (Vehicles)
        vehicle_list = world.get_actors().filter('vehicle.*')
        # Print all index corresponding to all traffic vehicles in scene (CarlaUE4)

        if args.duration == 0:
            label_duration = 0.5
        else:
            label_duration = args.duration

        if len(vehicle_list) == 0:
            print("NO VEHICLES")
        else:
            for index, vehicle in enumerate(vehicle_list, start=1):
                print(str(vehicle.attributes))
                world.debug.draw_string(vehicle.get_location(), str(vehicle.attributes["role_name"]), draw_shadow=False,
                                                    color=carla.Color(r=255, g=0, b=0), life_time=label_duration,
                                                    persistent_lines=True)
            
                print(str(vehicle_list) + "  " +  str(vehicle_list[0].attributes))

        if args.duration != 0:
            sys.exit()

        time.sleep(0.5)
except Exception as err_msg:
    print(str(err_msg))
    print("\nERROR CONNECTING TO CARLA")



    ################################################################################################
    # Once you see all index number, you can manually change its states and timimg.
    # Your signal control scripts.
