# Add PWD to the path
import os
import sys

print("Running tests/__init__.py")

current_dir = os.path.dirname(os.path.abspath(__file__))
# lib_dir = os.path.join(current_dir, "..")
lib_dir = os.path.join(current_dir, "../lib")
sys.path.append(lib_dir)

# import lib

# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))



