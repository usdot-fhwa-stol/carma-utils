# Add PWD to the path
import os
import sys

# TODO Cleanup
print("Running tests/__init__.py")

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "../lib"))
import lib
