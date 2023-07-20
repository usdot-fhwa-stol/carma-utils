import glob
import os
import sys
import time

import fnmatch
from os.path import expanduser

def find_file(pattern, path):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                result.append(os.path.join(root, name))
    return result 

def find_carla_egg():

    carla_egg_dir = os.getenv("VOI_CARLA_EGG_DIR")

    if not carla_egg_dir:
        print("\n[!!!] VOI_CARLA_EGG_DIR not set, source node_info.config")
        sys.exit()

    #this looks for the carla python API .egg file
    try:
        
        carla_egg_name = 'carla-0.9.10*' + str(sys.version_info.major) + '*-' + str('win-amd64' if os.name == 'nt' else 'linux-x86_64') + '.egg'
        print("Looking for CARLA egg: " + carla_egg_name)
        carla_egg_locations = find_file(carla_egg_name,carla_egg_dir)
        print("Found carla egg(s): " + str(carla_egg_locations))

        if len(carla_egg_locations) == 0:
            print("\nUNABLE TO FIND CARLA EGG")
            sys.exit()
        elif len(carla_egg_locations) == 1:
            carla_egg_to_use = carla_egg_locations[0]
        else:
            print("\nFound multiple carla egg files: ")
            for i,egg_found in enumerate(carla_egg_locations):
                print("[" + str(i+1) + "]    " + egg_found)

            egg_selected = input("\nSelect a carla egg file to use: ")

            try:
                egg_selected = int(egg_selected)
            except:
                print("\nInvalid selection, please try again")
                sys.exit()

            if (egg_selected <= len(carla_egg_locations)):
                carla_egg_to_use = carla_egg_locations[egg_selected-1]
            else:
                print("\nInvalid selection, please try again")
                sys.exit()

        return carla_egg_to_use 

    except IndexError:
        print("\nUNABLE TO FIND CARLA EGG")
        
        pass
