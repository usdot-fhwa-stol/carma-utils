import os
import sys

# Explicitly import library into test harness.
# Inspiration: https://github.com/Rhoynar/sample-python/tree/master/tests
current_dir = os.path.dirname(os.path.abspath(__file__))
lib_dir = os.path.join(current_dir, "../lib")
sys.path.insert(0, lib_dir)

# Load CARLA
from util.CarlaLoader import CarlaLoader
