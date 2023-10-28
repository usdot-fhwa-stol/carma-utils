import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
lib_dir = os.path.join(current_dir, "../lib")
sys.path.insert(0, lib_dir)

from util.CarlaLoader import CarlaLoader
